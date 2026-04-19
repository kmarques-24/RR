#include "estimator.h"
#include "utils.h"
#include "imu_service.h"
#include "hardware_encoders.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

#include "esp_system.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"
#include "esp_log.h"
#include "esp_event.h"

#define METERS_PER_COUNT (2.0f * M_PI * (CONFIG_WHEEL_RADIUS_MM / 1000.0f) / CONFIG_CPR)
// meters/count = (meters/revolution) / (counts/revolution)
// circumference travelled = distance travelled (no slip)

static const char *TAG = "Estimator";
bool estimator_initialized = false;

static SemaphoreHandle_t dataMutex; // protect while writing
static StaticSemaphore_t dataMutexBuffer;

// Local history for estimator to calculate position
static int16_t prevCountLeft;
static int16_t prevCountRight;

// Global instance
static odom_data_t odom_latest;

void update_odometry_msg(nav_msgs__msg__Odometry *odom_msg)
{
    // lock here too to avoid partial data read
    xSemaphoreTake(dataMutex, portMAX_DELAY);

    odom_msg->header.stamp.sec = odom_latest.timestamp.secs; 
    odom_msg->header.stamp.nanosec = odom_latest.timestamp.nanosecs;

    odom_msg->pose.pose.position.x = odom_latest.position.x;
    odom_msg->pose.pose.position.y = odom_latest.position.y;
    odom_msg->pose.pose.position.z = odom_latest.position.z;

    odom_msg->pose.pose.orientation.w = odom_latest.quat.w;
    odom_msg->pose.pose.orientation.x = odom_latest.quat.x;
    odom_msg->pose.pose.orientation.y = odom_latest.quat.y;
    odom_msg->pose.pose.orientation.z = odom_latest.quat.z;

    odom_msg->twist.twist.linear.x = odom_latest.twist_lin.x; 
    odom_msg->twist.twist.linear.y = odom_latest.twist_lin.y;
    odom_msg->twist.twist.linear.z = odom_latest.twist_lin.z;

    odom_msg->twist.twist.angular.x = odom_latest.twist_ang.x;
    odom_msg->twist.twist.angular.y = odom_latest.twist_ang.y;
    odom_msg->twist.twist.angular.z = odom_latest.twist_ang.z;

    xSemaphoreGive(dataMutex);
}

void init_estimator(void)
{
    ESP_LOGI(TAG, "Estimator enabled");

    odom_latest.quat.w = 1; // rest are 0 at startup

    prevCountLeft = left_encoder.count;
    prevCountRight = right_encoder.count;

    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
    // simple for now, but may need to more checks like ToF and IMU when filtering added
    estimator_initialized = true;
}

void estimator_task(void *pvParameter)
{
    while(1)
    {
        imu_data_t imu_data = {};  // zero initialize as fallback
        if (imu_initialized)
        {
            get_latest_imu(&imu_data);
        }

        // Grab both to avoid encoder ISR updating right as I'm reading left for example
        taskENTER_CRITICAL(&enc_mux);
        int16_t countL = left_encoder.count;
        int16_t countR = right_encoder.count;
        taskEXIT_CRITICAL(&enc_mux);

        float vel_left  = left_encoder.countVelocity  * METERS_PER_COUNT;  // m/s
        float vel_right = right_encoder.countVelocity * METERS_PER_COUNT;  // m/s
        
        // Lock odom_latest while it updates
        xSemaphoreTake(dataMutex, portMAX_DELAY);

        int16_t countDiffLeft = differenceWrapped(countL, prevCountLeft); // signed
        int16_t countDiffRight = differenceWrapped(countR, prevCountRight); // signed
        prevCountLeft = countL;
        prevCountRight = countR;

        float delta_left  = countDiffLeft  * METERS_PER_COUNT;  // meters traveled this step
        float delta_right = countDiffRight * METERS_PER_COUNT;
        float delta_s     = (delta_right + delta_left) / 2.0f;  // arc length of center
        //float delta_theta = (delta_right - delta_left) / (CONFIG_WHEEL_BASE_MM / 1000.0f);  // heading change

        float linear_vel  = (vel_right + vel_left) / 2.0f;   // twist.linear.x
        //float angular_vel = (vel_right - vel_left) / (CONFIG_WHEEL_BASE_MM / 1000.0f);  // twist.angular.z

        // not using delta_theta (to integrate to get theta) or angular_vel 
        // calculated via encoders above because IMU is more accurate for both
        // maybe fuse in future

        // get timestamp
        timespec_t time = getTime();
        odom_latest.timestamp.secs = time.secs;
        odom_latest.timestamp.nanosecs = time.nanosecs;

        float psi = imu_data.euler.yaw;
        odom_latest.position.x += delta_s * cosf(psi);
        odom_latest.position.y += delta_s * sinf(psi);
        odom_latest.position.z = 0; // can update later and integrate along heading too

        // Transform?
        odom_latest.quat.w = imu_data.quat.w;
        odom_latest.quat.x = imu_data.quat.x;
        odom_latest.quat.y = imu_data.quat.y;
        odom_latest.quat.z = imu_data.quat.z;
        
        odom_latest.twist_lin.x = linear_vel;
        odom_latest.twist_lin.y = 0; // always 0
        odom_latest.twist_lin.z = 0; // always 0
        
        // Transform?
        odom_latest.twist_ang.x = 0; // 0 for now
        odom_latest.twist_ang.y = 0; // 0 for now
        odom_latest.twist_ang.z = imu_data.ang_vel.z; // yaw rate

        xSemaphoreGive(dataMutex); // done updating

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

BaseType_t start_estimator(void)
{
    if (!estimator_initialized)
    {
        ESP_LOGE(TAG, "Estimator not initialized, cannot start service");
        return pdFAIL;
    }

    BaseType_t status;
    status = xTaskCreate(
        estimator_task,
        "estimator_task",
        4096,
        NULL,
        4, // should be after the IMU and Encoder
        NULL);
    
    if (status == pdPASS)
    {
        ESP_LOGI(TAG, "Estimator started!");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting estimator.");
    }
    return status;
}
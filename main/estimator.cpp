/*
We want estimator onboard so we can publish a single odometry message to the ROS2 client.
This saves bandwidth for many robots and keeps estimation fast. 

Using robot_localization ROS2 pkg running on a downstream estimator to fuse /odom and 
/imu/data can't run on esp32s3. Would have to be offboard so it doesn't suit our purposes. 
(For reference though, it would consume just x, y, x rate, yaw rate from /odom to avoid
treating placeholder 0s for values that can't be known as true odometry. This is also done 
by setting high covariance 1e9 for these fields to signal they're not known.)

TODO:
- Make proper onboard EKF that can handle radio data
- Fuse radio data to correct position
- Include error checks for radio initialization
*/

#include "estimator.h"
#include "utils.h"
#include "imu_service.h"
#include "hardware_encoders.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BNO08x.hpp" // only for euler conversion math

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

// IMU mounting rotation relative to base_link (base_link -> imu_link)
void transform_imu_to_base(const imu_data_t *in, imu_data_t *out);
/*
Note: ROS2 base link convention for forward-facing robot: x forward, y left, z up
1. Align IMU axis with robot axes
2. Rotate IMU to desired position
3. Read published orientation values rounded to nearest 0.707
4. This is the base_link -> imu_link transform. IMU data needs to be rotated by this.
5. Can also determine desired rotation and convert to quaternion
Below is a -90 degree rotation
*/
#define IMU_TO_BASE_QW  0.707f
#define IMU_TO_BASE_QX  0.0f
#define IMU_TO_BASE_QY  -0.707f
#define IMU_TO_BASE_QZ  0.0f

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

    if (!imu_initialized)
    {
        ESP_LOGW(TAG, "IMU not initialized. Using encoder-based orientation");
    }
    if (!encoders_initialized)
    {
        ESP_LOGE(TAG, "Estimator not initialized. Encoders must be initialized first");
        return;
    }

    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
    estimator_initialized = true;
}

void estimator_task(void *pvParameter)
{
    float psi = 0; // initial yaw = 0
    
    while(1)
    {
        // Transform IMU data from imu_link to base_link frame
        imu_data_t imu_data;
        imu_data_t imu_transformed;
        if (imu_initialized)
        {
            get_latest_imu(&imu_data);
            transform_imu_to_base(&imu_data, &imu_transformed);
        }
        
        // Grab both at same time - avoid encoder ISR updating right as I'm reading left, for example
        taskENTER_CRITICAL(&enc_mux);
        int16_t countL = left_encoder.count;
        int16_t countR = right_encoder.count;
        taskEXIT_CRITICAL(&enc_mux);

        // Wheel velocities in m/s
        float vel_left  = left_encoder.countVelocity  * METERS_PER_COUNT;  // m/s
        float vel_right = right_encoder.countVelocity * METERS_PER_COUNT;  // m/s
        
        // Protect odometry data in odom_latest from being read while it updates
        xSemaphoreTake(dataMutex, portMAX_DELAY);

        // Difference in encoder count since last estimator loop
        int16_t countDiffLeft = differenceWrapped(countL, prevCountLeft); // signed
        int16_t countDiffRight = differenceWrapped(countR, prevCountRight); // signed
        prevCountLeft = countL;
        prevCountRight = countR;

        // Total distance travelled
        float delta_left  = countDiffLeft  * METERS_PER_COUNT;  // meters traveled this step
        float delta_right = countDiffRight * METERS_PER_COUNT;
        float delta_s     = (delta_right + delta_left) / 2.0f;  // arc length of center
        
        if (!imu_initialized)
        {
            // Encoder-derived heading (yaw). Drifts due to slip and model inaccuracies.
            // The BNO08x (IMU) runs its own EKF to fuse heading (accel, gyro, mag), so we prefer to use this
            // directly unless the IMU isn't working
            float delta_psi = (delta_right - delta_left) / (CONFIG_WHEEL_BASE_MM / 1000.0f);  // heading change
            psi += delta_psi;

            // Encoder-derived change in heading (yaw rate). 
            float angular_vel = (vel_right - vel_left) / (CONFIG_WHEEL_BASE_MM / 1000.0f);  // twist.angular.z

            // Update orientation
            odom_latest.quat.w = cosf(psi * 0.5f);
            odom_latest.quat.x = 0; // unknown
            odom_latest.quat.y = 0; // unknown
            odom_latest.quat.z = sinf(psi * 0.5f);

            // Update rotation rates
            odom_latest.twist_ang.x = 0; // roll rate
            odom_latest.twist_ang.y = 0; // pitch rate
            odom_latest.twist_ang.z = angular_vel; // yaw rate
        }
        else
        {
            // Get heading
            psi = imu_transformed.euler.yaw;

            // Update orientation
            odom_latest.quat.w = imu_transformed.quat.w; 
            odom_latest.quat.x = imu_transformed.quat.x; 
            odom_latest.quat.y = imu_transformed.quat.y; 
            odom_latest.quat.z = imu_transformed.quat.z; 

            // Update rotation rates
            odom_latest.twist_ang.x = imu_transformed.ang_vel.x; // roll rate
            odom_latest.twist_ang.y = imu_transformed.ang_vel.y; // pitch rate
            odom_latest.twist_ang.z = imu_transformed.ang_vel.z; // yaw rate
        }
        
        float linear_vel  = (vel_right + vel_left) / 2.0f;   // twist.linear.x

        // Get timestamp
        timespec_t time = getTime();
        odom_latest.timestamp.secs = time.secs;
        odom_latest.timestamp.nanosecs = time.nanosecs;

        // Update position by integrating distance along heading
        odom_latest.position.x += delta_s * cosf(psi);
        odom_latest.position.y += delta_s * sinf(psi);
        odom_latest.position.z = 0; // can update later and integrate along heading too
        
        // Update velocities
        odom_latest.twist_lin.x = linear_vel;   // robot drives forward/backward
        odom_latest.twist_lin.y = 0;            // always 0 - robot can't move sideways
        odom_latest.twist_lin.z = 0;            // always 0  robot can't move upward

        xSemaphoreGive(dataMutex); // done updating

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

/**
 * @brief Transforms IMU data from imu_link frame into base_link frame.
 *
 * Applies the fixed mounting rotation described by IMU_TO_BASE_Q*. Vectors
 * (ang_vel, lin_accel, mag) are rotated by q_bi = q(base_link->imu_link).
 * The orientation quaternion is composed with the conjugate of q_bi to
 * convert "imu_link orientation in world" into "base_link orientation in
 * world". Euler angles are recomputed from the transformed quaternion so
 * they stay consistent.
 *
 * @param in   IMU data in imu_link frame.
 * @param out  IMU data in base_link frame. Timestamp and other non-vector
 *             fields are copied as-is.
 */
void transform_imu_to_base(const imu_data_t *in, imu_data_t *out)
{
    // Copy fields that don't need transformation (timestamp)
    out->timestamp = in->timestamp;

    // Rotate angular velocity: ω_base = q_bi * ω_imu * conj(q_bi)
    rotate_vec_by_quat(in->ang_vel.x, in->ang_vel.y, in->ang_vel.z,
                       IMU_TO_BASE_QW, IMU_TO_BASE_QX, IMU_TO_BASE_QY, IMU_TO_BASE_QZ,
                       &out->ang_vel.x, &out->ang_vel.y, &out->ang_vel.z);

    // Rotate linear acceleration: a_base = q_bi * a_imu * conj(q_bi)
        // Note: ignoring centripetal/tangential terms from IMU offset from base_link origin.
        // Valid approximation for ground robots at moderate angular rates.
    rotate_vec_by_quat(in->lin_accel.x, in->lin_accel.y, in->lin_accel.z,
                       IMU_TO_BASE_QW, IMU_TO_BASE_QX, IMU_TO_BASE_QY, IMU_TO_BASE_QZ,
                       &out->lin_accel.x, &out->lin_accel.y, &out->lin_accel.z);

    // Rotate magnetometer: mag_base = q_bi * mag_imu * conj(q_bi)
    rotate_vec_by_quat(in->mag.x, in->mag.y, in->mag.z,
                       IMU_TO_BASE_QW, IMU_TO_BASE_QX, IMU_TO_BASE_QY, IMU_TO_BASE_QZ,
                       &out->mag.x, &out->mag.y, &out->mag.z);

    // Compose orientation: q_wb = q_wi * conj(q_bi)
        // q_wi = q_wb * q_bi, so q_wb = q_wi * conj(q_bi).
    quat_multiply(in->quat.w, in->quat.x, in->quat.y, in->quat.z,
                  IMU_TO_BASE_QW, -IMU_TO_BASE_QX, -IMU_TO_BASE_QY, -IMU_TO_BASE_QZ,
                  &out->quat.w, &out->quat.x, &out->quat.y, &out->quat.z);

    // Recompute euler angles from transformed quaternion so they match
        // Standard ZYX (yaw-pitch-roll) extraction.
    bno08x_quat_t q_transformed;
    q_transformed.real = out->quat.w;
    q_transformed.i    = out->quat.x;
    q_transformed.j    = out->quat.y;
    q_transformed.k    = out->quat.z;

    bno08x_euler_angle_t euler_transformed;
    euler_transformed = q_transformed;
    out->euler.roll  = euler_transformed.x;
    out->euler.pitch = euler_transformed.y;
    out->euler.yaw   = euler_transformed.z;
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
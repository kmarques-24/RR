#include "include/imu_service.h"
#include "BNO08x.hpp"

// Standard includes
#include <stdio.h>
#include <string.h>
#include <math.h>

// FreeRTOS and ESP32 includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"
#include "esp_event.h"

// Polling and report frequency
#define POLLING_FREQ_HZ     250
#define POLLING_PERIOD_MS   (uint32_t)(1000 / POLLING_FREQ_HZ)
#define REPORT_FREQ_HZ      100
#define REPORT_PERIOD_US    (uint32_t)(1000000 / REPORT_FREQ_HZ) // beware integer div and order of operations

// IMU data
static BNO08x imu;
static imu_data_t imu_data_latest;
static SemaphoreHandle_t dataMutex; // protect while writing
static StaticSemaphore_t dataMutexBuffer;

bool imu_initialized = false;

// Holders
bno08x_quat_t quat;
bno08x_ang_vel_t omega;
bno08x_euler_angle_t euler;
static const char *TAG = "IMU";
//TaskHandle_t estimator_task_handle = NULL;

void update_imu_msg(sensor_msgs__msg__Imu *imu_msg)
{
    // lock here too to avoid partial data read
    xSemaphoreTake(dataMutex, portMAX_DELAY);

    imu_msg->orientation.w = imu_data_latest.quat.w; 
    imu_msg->orientation.x = imu_data_latest.quat.x; 
    imu_msg->orientation.y = imu_data_latest.quat.y; 
    imu_msg->orientation.z = imu_data_latest.quat.z; 

    imu_msg->angular_velocity.x = imu_data_latest.ang_vel.x; 
    imu_msg->angular_velocity.y = imu_data_latest.ang_vel.y; 
    imu_msg->angular_velocity.z = imu_data_latest.ang_vel.z; 
    
    imu_msg->linear_acceleration.x = imu_data_latest.lin_accel.x;
    imu_msg->linear_acceleration.y = imu_data_latest.lin_accel.y;
    imu_msg->linear_acceleration.z = imu_data_latest.lin_accel.z;

    xSemaphoreGive(dataMutex);
}

void get_latest_imu(imu_data_t *imu_data)
{
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    *imu_data = imu_data_latest;
    xSemaphoreGive(dataMutex);
}

void init_imu(void)
{
    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "IMU failed to initialize.");
        return;
    }

    if (imu.rpt.rv_gyro_integrated.enable(REPORT_PERIOD_US) != ESP_OK) 
    {
        ESP_LOGE(TAG, "rv_gyro_integrated failed"); 
        return;
    }
    if (imu.rpt.linear_accelerometer.enable(REPORT_PERIOD_US) != ESP_OK) 
    { 
        ESP_LOGE(TAG, "linear_accelerometer failed"); 
        return; 
    }
    if (imu.rpt.accelerometer.enable(REPORT_PERIOD_US) != ESP_OK) 
    { 
        ESP_LOGE(TAG, "accelerometer failed"); 
        return; 
    }
    if (imu.rpt.cal_magnetometer.enable(REPORT_PERIOD_US) != ESP_OK) 
    { 
        ESP_LOGE(TAG, "cal_magnetometer failed"); 
        return; 
    }

    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
    imu_initialized = true;
    ESP_LOGI(TAG, "IMU initialized successfully");
}

void imu_task(void *pvParameter)
{
    ESP_LOGI("IMU_TASK", "IMU task started on core %d", xPortGetCoreID());
    //UBaseType_t stack_remaining;
    while(1)
    {
        // block until new report is detected
        if (imu.data_available())
        {
            //ESP_LOGI("IMU_TASK", "data available");
            // Lock while imu_data_latest updates
            xSemaphoreTake(dataMutex, portMAX_DELAY);

            // get timestamp
            timespec_t time = getTime();
            imu_data_latest.timestamp.secs = time.secs;
            imu_data_latest.timestamp.nanosecs = time.nanosecs;

            //stack_remaining = uxTaskGetStackHighWaterMark(NULL);
            //ESP_LOGI(TAG, "Stack remaining: %u", stack_remaining);

            // Get the latest report from Gyro (orientation & angular velocity, already filtered & fused)
            if (imu.rpt.rv_gyro_integrated.has_new_data())
            {
                // Orientation
                imu.rpt.rv_gyro_integrated.get(quat, omega);
                imu_data_latest.quat.w = quat.real;
                imu_data_latest.quat.x = quat.i;
                imu_data_latest.quat.y = quat.j;
                imu_data_latest.quat.z = quat.k;

                imu_data_latest.ang_vel.x = omega.x;
                imu_data_latest.ang_vel.y = omega.y;
                imu_data_latest.ang_vel.z = omega.z;

                euler = quat; // see overloaded = operator in BNO08xGlobalTypes.hpp
                // ESP_LOGI(TAG, "Roll: %f, Pitch: %f, Yaw: %f", euler.x, euler.y, euler.z);

                imu_data_latest.euler.roll = euler.x;
                imu_data_latest.euler.pitch = euler.y;
                imu_data_latest.euler.yaw = euler.z;
            }

            // Get linear acceleration data (gravity vector subtracted)
            if (imu.rpt.linear_accelerometer.has_new_data())
            {
                bno08x_accel_t lin_accel = imu.rpt.linear_accelerometer.get();
                // ESP_LOGI(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
                imu_data_latest.lin_accel.x = lin_accel.x;
                imu_data_latest.lin_accel.y = lin_accel.y;
                imu_data_latest.lin_accel.z = lin_accel.z;
            }

            // Get calibrated magnetic field
            if (imu.rpt.cal_magnetometer.has_new_data())
            {
                bno08x_magf_t mag_data = imu.rpt.cal_magnetometer.get();
                // ESP_LOGI(TAG, "Mag Field: (magx: %.2f magy: %.2f magz: %.2f)[T]", mag_data.x, mag_data.y, mag_data.z);
                imu_data_latest.mag.x = mag_data.x;
                imu_data_latest.mag.y = mag_data.y;
                imu_data_latest.mag.z = mag_data.z;
            }

            xSemaphoreGive(dataMutex); // done updating
        }
        //vTaskDelay(pdMS_TO_TICKS(POLLING_PERIOD_MS)); 
            // library isn't set up for polling - have to have HINT
            // isAvailable is blocking at the report frequency (100 Hz), so don't need VTaskDelay
    }
}

BaseType_t imu_service(void)
{
    if (!imu_initialized)
    {
        ESP_LOGE(TAG, "IMU not initialized, cannot start service");
        return pdFAIL;
    }
    
    BaseType_t status;
    status = xTaskCreate(
        imu_task,
        "imu_task",
        4096,
        NULL,
        5,
        NULL);
    
    if (status == pdPASS)
    {
        ESP_LOGI(TAG, "IMU service started!");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting IMU service.");
    }
    return status;
}
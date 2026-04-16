#include "include/imu_service.h"
#include "include/rr_os_service.h"
#include <stdio.h>
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "include/wireless_driving.h"

// Polling and report frequency
#define POLLING_FREQ_HZ     250
#define POLLING_PERIOD_MS   (uint32_t)(1/POLLING_FREQ_HZ*1000)
#define REPORT_FREQ_HZ      100
#define REPORT_PERIOD_US    (uint32_t)(1/REPORT_FREQ_HZ*1000000)

// IMU data
static BNO08x imu;
imu_data_t imu_data_latest;

// Holders
bno08x_quat_t quat;
bno08x_ang_vel_t omega;
bno08x_euler_angle_t euler;

static const char *TAG = "IMU";

void init_imu()
{
    ESP_LOGI(TAG, "IMU enabled");
    // initialize imu
    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "Init failure, returning from main.");
        return;
    }
    // 10000 us = 10 ms report interval (100 Hz)
        // changed from 100,000us == 100ms report interval
    imu.rpt.rv_gyro_integrated.enable(REPORT_PERIOD_US);
    imu.rpt.linear_accelerometer.enable(REPORT_PERIOD_US);
    imu.rpt.accelerometer.enable(REPORT_PERIOD_US);
    imu.rpt.cal_magnetometer.enable(REPORT_PERIOD_US);
    ESP_LOGI(TAG, "IMU enabled");
}

void imu_loop(void *pvParameter)
{
    UBaseType_t stack_remaining;
    while(1)
    {
        // get timestamp
        uint32_t imu_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
        imu_data_latest.timestamp = imu_time_ms;

        // block until new report is detected
        if (imu.data_available())
        {
            stack_remaining = uxTaskGetStackHighWaterMark(NULL);
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
        }
        else 
        {
            //ESP_LOGI(TAG, "No data available");
        }
        vTaskDelay(pdMS_TO_TICKS(POLLING_PERIOD_MS)); // polling to save a pin
        // changed from 1000. Can return to this to debug. Should be less than report interval
    }
}

BaseType_t imu_service(void)
{
    BaseType_t status;
    status = xTaskCreate(
        imu_loop,
        "imu_loop",
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
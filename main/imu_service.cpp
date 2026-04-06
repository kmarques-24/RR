#include "include/imu_service.h"
#include "include/rr_os_service.h"
#include <fcntl.h>    // for open(), O_WRONLY, O_CREAT, etc.
#include <unistd.h>   // for close(), write(), etc.
#include <stdio.h>
#include "esp_spiffs.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "include/wireless_driving.h"



static BNO08x imu;
bno08x_quat_t quat;
bno08x_euler_angle_t euler;
bno08x_ang_vel_t omega;
char imu_buf[512];
int imu_buf_ret;

// imu data variables
float roll = 0, pitch = 0, yaw = 0;
float linx = 0, liny = 0, linz = 0;
float angx = 0, angy = 0, angz = 0;
float magx = 0, magy = 0, magz = 0;

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
    imu.rpt.rv_gyro_integrated.enable(100000UL); // 100,000us == 100ms report interval
    imu.rpt.linear_accelerometer.enable(100000UL);
    imu.rpt.accelerometer.enable(100000UL);
    imu.rpt.cal_magnetometer.enable(100000UL);
    ESP_LOGI(TAG, "IMU enabled fr");
}

void imu_loop(void *pvParameter)
{
    UBaseType_t stack_remaining;
    while(1)
    {
        // clear buf 
        imu_buf[0] = '\0';
        // get timestamp
        int32_t imu_time_ms = (int32_t)(esp_timer_get_time() / 1000);
        imu_time_to_buf(imu_time_ms);
        // block until new report is detected
        if (imu.data_available())
        {
            stack_remaining = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "Stack remaining: %u", stack_remaining);
            
            // Get the latest report from Gyro
            if (imu.rpt.rv_gyro_integrated.has_new_data())
            {
                imu.rpt.rv_gyro_integrated.get(quat, omega);
                euler = quat;
                // ESP_LOGI(TAG, "Roll: %f, Pitch: %f, Yaw: %f", euler.x, euler.y, euler.z);
                roll = euler.x;
                pitch = euler.y;
                yaw = euler.z;
            }

            // Get linear acceleration data
            if (imu.rpt.linear_accelerometer.has_new_data())
            {
                bno08x_accel_t lin_accel = imu.rpt.accelerometer.get();
                // ESP_LOGI(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
                linx = lin_accel.x;
                liny = lin_accel.y;
                linz = lin_accel.z;
            }

            // Get Angular acceleration data
            if (imu.rpt.accelerometer.has_new_data())
            {
                bno08x_accel_t ang_accel = imu.rpt.accelerometer.get();
                // ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
                angx = ang_accel.x;
                angy = ang_accel.y;
                angz = ang_accel.z;
            }
            // Get calibrated magnetic field
            if (imu.rpt.cal_magnetometer.has_new_data())
            {
                bno08x_magf_t mag_data = imu.rpt.cal_magnetometer.get();
                // ESP_LOGI(TAG, "Mag Field: (magx: %.2f magy: %.2f magz: %.2f)[T]", mag_data.x, mag_data.y, mag_data.z);
                magx = mag_data.x;
                magy = mag_data.y;
                magz = mag_data.z;
            }
        }
        else 
        {
            ESP_LOGI(TAG, "No data available");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // write to buf
        gyro_data_to_buf(roll);
        gyro_data_to_buf(yaw);
        gyro_data_to_buf(pitch);
        data_to_buf(linx);
        data_to_buf(liny);
        data_to_buf(linz);
        data_to_buf(angx);
        data_to_buf(angy);
        data_to_buf(angz);
        data_to_buf(magx);
        data_to_buf(magy);
        data_to_buf(magz);
        size_t len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "\n")) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
        // write from buf to data file
        if (autonomous_mode) {
            imu_buf_to_text();
        }
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
        ESP_LOGI(TAG, "service started");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting the service");
    }
    return status;
}

void imu_buf_to_text() {  
    // write from buf to file
    FILE *f = fopen("/storage/IMU_data.txt", "a");  // "a" to append
    if (f == NULL) {
    ESP_LOGE("FILE", "Failed to open file for writing");
    } 
    else {
        fwrite(imu_buf, 1, strlen(imu_buf), f);  // write the buffer
        fclose(f);
        ESP_LOGI("FILE", "Data written to file");
    }   
}

// converts time in ms to seconds and ms and writes these to the buffer
void imu_time_to_buf(int32_t time_ms) {
    int32_t sec = time_ms / 1000;
    int32_t ms = time_ms % 1000;
    // write to buf
    size_t len = strlen(imu_buf);
    // writing seconds
    if (sec < 10) { // want to write 000x seconds
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "000%ld", sec)) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else if (sec >= 10 && sec < 100) { // want to write 00xx seconds
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "00%ld", sec)) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else if (sec >= 100 && sec < 1000) { // want to write 0xxx seconds
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "0%ld", sec)) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else if (sec >= 1000 && sec < 10000) { // xxxx seconds
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "%ld", sec)) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else if (sec >= 10000) { // warning
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "OVER")) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }

    // writing ms
    if (ms < 10) { // want to write 00x ms
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "00%ld", ms)) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else if (ms >= 10 && ms < 100) { // want to write 0xx ms
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "0%ld", ms)) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else if (ms >= 100 && ms < 1000) { // xxx ms
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "%ld", ms)) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else if (ms >= 1000) { // warning
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "OVR")) < 0) {
            ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
}

void gyro_data_to_buf(float data) {
    float abs_data = fabs(data);
    size_t len;
    if (abs_data > 1) { // warning
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "OVERMAXIM")) < 0) {
               ESP_LOGE(TAG, "Failed to write to buffer");
        }
        // for debugging
        // len = strlen(imu_buf);
        // if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "%f", abs_data)) < 0) {
        //        ESP_LOGE(TAG, "Failed to write to buffer");
        // }
        return;
    }
    if (data < 0) { // negative
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "0")) < 0) {
               ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else { // positive
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "1")) < 0) {
               ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    if (abs_data <= 1) { 
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "%f", abs_data)) < 0) {
               ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
}

void data_to_buf(float data) {
    float abs_data = fabs(data);
    size_t len;

    if (abs_data >= 100) { // shouldn't be, but just in case we want to signal error and not mess up formatting
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "OVRMAX")) < 0) {
                    ESP_LOGE(TAG, "Failed to write to buffer");
        }
        return;
    }

    if (data < 0) { // negative sign
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "0")) < 0) {
                    ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else {
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "1")) < 0) {
                    ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    if (abs_data < 10) { // 0x.xx
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "0%.2f", abs_data)) < 0) {
                    ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
    else if (abs_data >= 10 && abs_data < 100) {
        len = strlen(imu_buf);
        if ((imu_buf_ret = snprintf(imu_buf + len, sizeof(imu_buf) - len, "%.2f", abs_data)) < 0) {
                    ESP_LOGE(TAG, "Failed to write to buffer");
        }
    }
}
// OS and C Headers
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

// ESP-IDF Headers
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"

// FreeRTOS Headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Project Headers
#include "hardware_motors.h"
#include "uros_service.h"
#include "imu_service.h"
#include "hardware_encoders.h"
#include "controller.h"
#include "estimator.h"
#include "tof_service.h"
#include "rr_os_service.h"
#include "radio_service.h"
#include "twai_service.h"
#include "led.h"

static const char *TAG = "MAIN";

// track wifi connection to computer AP (access point)
// not static so that uros_service can see it

void mount_spiffs();
void initialise(rr_status_t rr_status); 
void test_drive_code();

// Compile using C linkage rules so app_main keeps its name. Required for ESP-IDF
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main");
    
    // Creating events queue (Set to True when you want a service working)
    // rr_status defined in rr_os_service.cpp
    rr_status.wifi_enabled = true;
    rr_status.encoder_enabled = true;
    rr_status.estimator_enabled = true;
    rr_status.imu_enabled = true;
    rr_status.key_control_enabled = true;
    rr_status.tof_enabled = true;

    rr_status.connected = false;
    rr_status.twai_active = false;
    rr_status.led_enabled = false;
    rr_status.radio_enabled = false;

    // Mount spiffs
    mount_spiffs();

    // Initialize peripherals
    initialise(rr_status);
    //uros_service(); // just test this for now

    // Loop forever to keep spiffs mounted
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void test_drive_code() {
    while(1){
        speed_callback(512, 512);
    } 
}

void mount_spiffs() {
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/storage",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t result = esp_vfs_spiffs_register(&config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(result));
        return; 
    }
}

void initialise(rr_status_t rr_status)
{
    if (rr_status.encoder_enabled)
    {
        ESP_LOGI(TAG, "Encoder service starting");
        init_encoders();
        encoder_service();
    }
    if (rr_status.drive_enabled)
    {
        ESP_LOGI(TAG, "Motor setup starting");
        initialise_drivetrain();
    }
    if (rr_status.estimator_enabled)
    {
        init_estimator();
        start_estimator();
        ESP_LOGI(TAG, "Estimator service starting");
    }
    if (rr_status.imu_enabled)
    {
        init_imu();
        imu_service();
        ESP_LOGI(TAG, "IMU service starting");
    }
    if (rr_status.key_control_enabled)
    {
        ESP_LOGI(TAG, "Controller service starting");
        init_controller();
        start_controller();
    }
    if (rr_status.tof_enabled)
    {
        ESP_LOGI(TAG, "ToF service starting");
        init_tof_sensor();
        tof_service();
    }
    if (rr_status.uros_enabled)
    {
        ESP_LOGI(TAG, "MicroROS service starting");
        uros_service();
    }

    // Not using these
    if (rr_status.led_enabled)
    {
        initialise_led();
        set_led_color(INDEPENDENT_COLOR);
        twai_interrupt_init();
    }
    if (rr_status.radio_enabled)
    { 
        //initialise_radio();
    } 
    
    //launch_rr_os_service(); // calls initialize_events()
}
/*
Instructions: 

1. Connect laptop to router
    - open http://192.168.8.1/ to see connect clients (laptop, esp32)
2. Open new terminal
    - navigate to local repo RR
    - run: code . (to open ESP-IDF project in VScode)
3. Open VScode terminal in ESP-IDF project
    - run: source $HOME/esp/v5.1.2/esp-idf/export.sh
4. Open new terminal
    - navigate to ~/ros2_ws
    - run: source /opt/ros/humble/setup.bash
    - run: source install/local_setup.bash
    - run: code . (to open ROS2 project in VScode)
5. Open VScode terminal in ROS2 project
    - run sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
    - this starts microROS agent
6. Open second VScode terminal in ROS2 project
    - run: ros2 topic echo /{topic}
    - this helps debug data I should see from publishers & to subscribers
7. As needed in ESP-IDF project VScode terminal:
    - run idf.py menuconfig to set project variables and save config
    - run idf.py reconfigure to regenerate
    - run idf.py set-target esp32s3 to resolve linkage errors
8. Run idf.py build, idf.py flash, idf.py monitor
    - monitor terminals and debug


*/

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
void initialise(void); 
void test_drive_code();

// Compile using C linkage rules so app_main keeps its name. Required for ESP-IDF
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main");
    ESP_LOGI("RESET", "reset reason: %d", esp_reset_reason());
    
    // Creating events queue (Set to True when you want a service working)
    // rr_status defined in rr_os_service.cpp
    rr_status.wifi_enabled = true;

    rr_status.tof_enabled = false;      // verified working
    rr_status.imu_enabled = false;      // verified working
    rr_status.drive_enabled = true;     // verified working

    rr_status.encoder_enabled = true;
    rr_status.key_control_enabled = false;
    rr_status.estimator_enabled = false;
    rr_status.uros_enabled = true;

    rr_status.connected = false;
    rr_status.twai_active = false;
    rr_status.led_enabled = false;
    rr_status.radio_enabled = false;

    // Mount spiffs
    mount_spiffs();

    // Initialize peripherals
    initialise();
    //uros_service(); // just test this for now

    // Debug motor test - must call after initializing
    //speed_callback(800, 800); // success

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

void initialise(void)
{
    if (rr_status.tof_enabled)
    {
        init_tof_sensor();
        if (tof_service() != pdPASS)
        {
            // disable so uros doesn't try to publish
            rr_status.tof_enabled = false;  
        }
        else
        {
            ESP_LOGI(TAG, "ToF service starting");
        }
    }

    if (rr_status.imu_enabled)
    {
        init_imu();
        if (imu_service() != pdPASS)
        {
            rr_status.imu_enabled = false;
        }
        else
        {
            ESP_LOGI(TAG, "IMU service starting");
        }
    }

    if (rr_status.drive_enabled)
    {
        ESP_LOGI(TAG, "Motor setup starting");
        initialise_drivetrain();
    }

    if (rr_status.key_control_enabled)
    {
        ESP_LOGI(TAG, "Controller service starting");
        init_controller();
        start_controller();
    }

        if (rr_status.encoder_enabled)
    {
        ESP_LOGI(TAG, "Encoder service starting");
        init_encoders();
        encoder_service();
    }
    if (rr_status.estimator_enabled)
    {
        init_estimator();
        start_estimator();
        ESP_LOGI(TAG, "Estimator service starting");
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

    // MicroROS enabled last
    if (rr_status.uros_enabled)
    {
        ESP_LOGI(TAG, "MicroROS service starting");
        uros_service();
    }
    
    //launch_rr_os_service(); // calls initialize_events()
}
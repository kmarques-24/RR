#include "include/hardware_drivetrain.h"
#include "include/wireless_driving.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <stdio.h>
#include <stdbool.h>
#include "include/encoder.h"
#include "include/imu_service.h"

// Motor speed definitions
// #define FULL_STOP 0
// #define DRIVE_SPEED 512
// #define TURN_SPEED 384

bool autonomous_mode = false; // Tracks if autonomous mode is active

void autonomous_control_loop(void *pvParameters) {
    while (autonomous_mode) {
        esp_task_wdt_reset(); // Prevent watchdog timeout
        
        // Control logic 
        speed_callback(DRIVE_SPEED, DRIVE_SPEED - 86);
                // 2. Move forward at ~50% speed for 2 seconds
        // vTaskDelay(5000 / portTICK_PERIOD_MS);

        // // 4. Spin in place (left forward, right backward)
        // speed_callback(-(TURN_SPEED), (TURN_SPEED));
        // vTaskDelay(3000 / portTICK_PERIOD_MS);

        // // move backwards
        // speed_callback(-DRIVE_SPEED, -DRIVE_SPEED);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        
        vTaskDelay(100 / portTICK_PERIOD_MS); // Yield to other tasks
    }
    
    // Cleanup before exit
    speed_callback(FULL_STOP, FULL_STOP);
    vTaskDelete(NULL); // Self-terminate
}

void process_drive_command(int command) {
    switch(command) {
        case 0: // STOP
            speed_callback(FULL_STOP, FULL_STOP);
            break;
            
        case 1: // FORWARD
            if (!autonomous_mode) speed_callback(DRIVE_SPEED, DRIVE_SPEED);
            break;
            
        case 2: // BACKWARD
            if (!autonomous_mode) speed_callback(-DRIVE_SPEED, -DRIVE_SPEED);
            break;
            
        case 4: // LEFT
            if (!autonomous_mode) speed_callback(-TURN_SPEED, TURN_SPEED);
            break;
            
        case 8: // RIGHT
            if (!autonomous_mode) speed_callback(TURN_SPEED, -TURN_SPEED);
            break;

        static TaskHandle_t autonomous_task_handle = NULL;

        case 9: // Toggle autonomous mode
            autonomous_mode = !autonomous_mode;
            if (autonomous_mode) {
                xTaskCreate(
                    autonomous_control_loop,
                    "autonomous_task",
                    4096,  
                    NULL,
                    4,     // Medium priority (below WiFi/imu/encoder)
                    &autonomous_task_handle  // Store handle
                );
                // imu_service();
                // encoder_service();
            } else {
                speed_callback(FULL_STOP, FULL_STOP);
                
                // Delete task if running
                if (autonomous_task_handle != NULL) {
                    vTaskDelete(autonomous_task_handle);
                    autonomous_task_handle = NULL;
                }
            }
            break;
                    
        default:
            speed_callback(FULL_STOP, FULL_STOP);
            break;
    }
}
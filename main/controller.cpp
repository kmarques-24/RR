// Project headers
#include "controller.h"
#include "hardware_encoders.h"
#include "hardware_motors.h"

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
#include "esp_timer.h"

#include <geometry_msgs/msg/twist.h>  

#define Kp 64.0f      // originally 0.2 for RPM -> 0.2 * 60 / (2*pi*0.03) = 
#define Kd 0.0f       // originally 0 for RPM
#define Ki 318.0f     // originally 1 for RPM -> 1 * 60 / (2*pi*0.03) = 
#define WINDUP_CAP 1000.0f
#define STOP_THRESHOLD 0.03f  // m/s -> below this is considered stopped

#define METERS_PER_COUNT (2.0f * M_PI * (CONFIG_WHEEL_RADIUS_MM / 1000.0f) / CONFIG_CPR)
// counts/s * (2*pi*m) * rev/count = m/s
// counts/s * rev/count * 60 s/min = RPM

static float dt;
static int64_t prevTime;

static const char *TAG = "Controller";

float setpoint_linvel_x; // desired speed
float setpoint_angvel_z; // desired yaw rate

motor_controller_t left_motor_ctrl;
motor_controller_t right_motor_ctrl;

static SemaphoreHandle_t dataMutex; // protect while writing
static StaticSemaphore_t dataMutexBuffer;

void drive_commanded_twist(const geometry_msgs__msg__Twist *twist_msg)
{
    // Convert twist to left/right motor velocities
    // Protect so nothing reads setpoint vals while I set
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    setpoint_linvel_x  = twist_msg->linear.x;
    setpoint_angvel_z = twist_msg->angular.z;
    xSemaphoreGive(dataMutex);
}

void init_controller(void)
{
    ESP_LOGI(TAG, "Controller enabled");
    // left motor and right motor controller fields are 0 already,
    // but memset is useful if we need to reset
    memset(&left_motor_ctrl,  0, sizeof(motor_controller_t));
    memset(&right_motor_ctrl, 0, sizeof(motor_controller_t));
    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
}

void controller_task(void *pvParameter)
{
    float wheel_base = CONFIG_WHEEL_BASE_MM / 1000.0f;  // m

    while(1)
    {
        // No critical block here. To know when to use, ask:
        // "could something (e.g. encoder ISR) interrupt to change my values halfway through my calcs?"
        // all these vals are task based, so no (?)
        int64_t now = esp_timer_get_time(); // time in us
        dt = (now - prevTime)/1000000.0f; // dt in s 
        prevTime = now;

        // Skip if junk dt
        if (dt <= 0.0f) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Protect before using
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        float lin = setpoint_linvel_x;
        float ang = setpoint_angvel_z;
        xSemaphoreGive(dataMutex);

        // Differential drive inverse kinematics
        float vel_left_des  = lin - (ang * wheel_base / 2.0f);  // m/s
        float vel_right_des = lin + (ang * wheel_base / 2.0f);  // m/s

        // Current speed in counts/s
        float vel_left_curr = left_encoder.countVelocity * METERS_PER_COUNT;
        float vel_right_curr = right_encoder.countVelocity * METERS_PER_COUNT;
        
        left_motor_ctrl.error = vel_left_des - vel_left_curr; // convention always desired - current
        right_motor_ctrl.error = vel_right_des - vel_right_curr;

        // Multiplying dt makes errorSum time consistent if dt varies
        // Kept here since controller dt could vary with scheduling
        left_motor_ctrl.errorSum += left_motor_ctrl.error * dt;
        right_motor_ctrl.errorSum += right_motor_ctrl.error * dt;
        if (left_motor_ctrl.errorSum > WINDUP_CAP) left_motor_ctrl.errorSum = WINDUP_CAP;
        if (left_motor_ctrl.errorSum < -WINDUP_CAP) left_motor_ctrl.errorSum = -WINDUP_CAP;
        if (right_motor_ctrl.errorSum > WINDUP_CAP) right_motor_ctrl.errorSum = WINDUP_CAP;
        if (right_motor_ctrl.errorSum < -WINDUP_CAP) right_motor_ctrl.errorSum = -WINDUP_CAP;

        int32_t left_duty = (int32_t)round((Kp * left_motor_ctrl.error) 
                        + (Kd * (left_motor_ctrl.error - left_motor_ctrl.prevError)/dt) 
                        + (Ki * left_motor_ctrl.errorSum));
        left_motor_ctrl.prevError = left_motor_ctrl.error;

        int32_t right_duty = (int32_t)round((Kp * right_motor_ctrl.error) 
                        + (Kd * (right_motor_ctrl.error - right_motor_ctrl.prevError)/dt) 
                        + (Ki * right_motor_ctrl.errorSum));
        right_motor_ctrl.prevError = right_motor_ctrl.error;

        // If setpoint is 0 and I'm basically at 0, stop
        if (fabsf(vel_left_des) < 0.001f && fabsf(vel_left_curr) <= STOP_THRESHOLD)
        {
            left_duty = 0;
            left_motor_ctrl.errorSum = 0; // don't holdover!
        }
        if (fabsf(vel_right_des) < 0.001f && fabsf(vel_right_curr) <= STOP_THRESHOLD)
        {
            right_duty = 0;
            right_motor_ctrl.errorSum = 0; // don't holdover!
        }

        // speed function already pwm capped
        speed_callback(left_duty, right_duty);
        
        vTaskDelay(pdMS_TO_TICKS(10)); // prev 20 (50 Hz)
    }
}

BaseType_t start_controller(void)
{
    BaseType_t status;
    status = xTaskCreate(
        controller_task,
        "controller_task",
        4096,
        NULL,
        4, // should be with estimator after IMU & encoder
        NULL);
    
    if (status == pdPASS)
    {
        ESP_LOGI(TAG, "Controller started!");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting controller.");
    }
    return status;
}

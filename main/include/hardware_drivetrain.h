#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configuration constants
#define SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQ 5000
#define PWM_RES LEDC_TIMER_10_BIT

// Motor pin configuration (from Kconfig)
#define LEFT_MOTOR_A CONFIG_LEFT_MOTOR_A
#define LEFT_MOTOR_B CONFIG_LEFT_MOTOR_B
#define RIGHT_MOTOR_A CONFIG_RIGHT_MOTOR_A
#define RIGHT_MOTOR_B CONFIG_RIGHT_MOTOR_B

// Motor structure definition
typedef struct {
    gpio_num_t pwma;
    gpio_num_t pwmb;
    ledc_channel_t channela;
    ledc_channel_t channelb;
} motor_t;

// External declarations for global variables
extern ledc_timer_config_t ledc_timer;
extern motor_t left_motor;
extern motor_t right_motor;

/**
 * @brief Initializes a motor with PWM configuration
 * @param m Motor structure containing pin and channel info
 */
void initialise_motor(motor_t m);

/**
 * @brief Sets speed and direction for a motor
 * @param m Motor to control
 * @param speed PWM duty cycle (0-1023)
 * @param forward Direction (true = forward, false = reverse)
 */
void set_motor_speed(motor_t m, int speed, bool forward);

/**
 * @brief Callback for velocity control
 * @param left_velocity Left motor velocity (-1023 to 1023)
 * @param right_velocity Right motor velocity (-1023 to 1023)
 * Negative values indicate reverse direction
 */
void speed_callback(int32_t left_velocity, int32_t right_velocity);

/**
 * @brief Initializes the complete drivetrain system
 * Configures PWM timer and both motors
 */
void initialise_drivetrain(void);

#ifdef __cplusplus
}
#endif
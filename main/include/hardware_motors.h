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

// Motor pin configuration
// from kconfig

// Motor structure definition
typedef struct {
    gpio_num_t pwm_m1;
    gpio_num_t pwm_m2;
    ledc_channel_t channel1;
    ledc_channel_t channel2;
} motor_config_t;

/**
 * @brief Initializes a motor with PWM configuration
 * @param m Motor structure containing pin and channel info
 */
void initialise_motor(motor_config_t m);

/**
 * @brief Sets speed and direction for a motor
 * @param m Motor to control
 * @param speed PWM duty cycle (0-1023)
 * @param forward Direction (true = forward, false = reverse)
 */
void set_motor_speed(motor_config_t m, int speed, bool forward);

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
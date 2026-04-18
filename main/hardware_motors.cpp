#include "include/hardware_motors.h"
// TODO: Brushless config if defined

// Global variable definitions
ledc_timer_config_t ledc_timer = {
    .speed_mode = SPEED_MODE,
    .duty_resolution = PWM_RES,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK,
    //.deconfigure = false // removed - ESP-IDF version does not have this field, should be implicitly false
};

motor_config_t left_motor_config = {
    .pwm_m1 = (gpio_num_t)CONFIG_LEFT_MOTOR_M1,
    .pwm_m2 = (gpio_num_t)CONFIG_LEFT_MOTOR_M2,
    .channel1 = LEDC_CHANNEL_1,
    .channel2 = LEDC_CHANNEL_0
};

motor_config_t right_motor_config = {
    .pwm_m1 = (gpio_num_t)CONFIG_RIGHT_MOTOR_M1,
    .pwm_m2 = (gpio_num_t)CONFIG_RIGHT_MOTOR_M2,
    .channel1 = LEDC_CHANNEL_2,
    .channel2 = LEDC_CHANNEL_3
};

void initialise_drivetrain()
{
    ESP_LOGD("Drivetrain", "Initialising LEDC");
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_LOGD("Drivetrain", "Initialising motors");
    initialise_motor(left_motor_config);
    initialise_motor(right_motor_config);
}

void initialise_motor(motor_config_t m)
{
    ledc_channel_config_t ledc_channel_cf = {};
    ledc_channel_cf.gpio_num = m.pwm_m1;            // pin set
    ledc_channel_cf.speed_mode = SPEED_MODE;
    ledc_channel_cf.channel = m.channel1;           // channel set
    ledc_channel_cf.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_cf.timer_sel = LEDC_TIMER_0;
    ledc_channel_cf.duty = 0;
    ledc_channel_cf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cf));

    ledc_channel_cf.gpio_num = m.pwm_m2;
    ledc_channel_cf.channel = m.channel2;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cf));
}

void set_motor_speed(motor_config_t m, int speed, bool forward)
{
    if (speed > 1023) speed = 1023; //sets max speed to 1023 for PMW
    
    if (forward)
    {
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channel2, speed));
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channel1, 0));
    }
    else
    {
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channel2, 0));
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channel1, speed));
    }
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, m.channel1));
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, m.channel2));
}

void speed_callback(int32_t left_duty, int32_t right_duty)
{
    // Positive values = forward, negative values = backward
    set_motor_speed(left_motor_config, abs(left_duty), left_duty >= 0);
    set_motor_speed(right_motor_config, abs(right_duty), right_duty >= 0);
}
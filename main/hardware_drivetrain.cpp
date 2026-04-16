#include "include/hardware_drivetrain.h"

// Global variable definitions
ledc_timer_config_t ledc_timer = {
    .speed_mode = SPEED_MODE,
    .duty_resolution = PWM_RES,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK,
    //.deconfigure = false // removed - ESP-IDF version does not have this field, should be implicitly false
};

motor_t left_motor = {
    .pwma = (gpio_num_t)LEFT_MOTOR_A,
    .pwmb = (gpio_num_t)LEFT_MOTOR_B,
    .channela = LEDC_CHANNEL_1,
    .channelb = LEDC_CHANNEL_0
};

motor_t right_motor = {
    .pwma = (gpio_num_t)RIGHT_MOTOR_A,
    .pwmb = (gpio_num_t)RIGHT_MOTOR_B,
    .channela = LEDC_CHANNEL_2,
    .channelb = LEDC_CHANNEL_3
};

void initialise_drivetrain()
{
    ESP_LOGD("Drivetrain", "Initialising LEDC");
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_LOGD("Drivetrain", "Initialising motors");
    initialise_motor(left_motor);
    initialise_motor(right_motor);
}

void initialise_motor(motor_t m)
{
    ledc_channel_config_t ledc_channel_cf = {};
    ledc_channel_cf.gpio_num = m.pwma;
    ledc_channel_cf.speed_mode = SPEED_MODE;
    ledc_channel_cf.channel = m.channela;
    ledc_channel_cf.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_cf.timer_sel = LEDC_TIMER_0;
    ledc_channel_cf.duty = 0;
    ledc_channel_cf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cf));

    ledc_channel_cf.gpio_num = m.pwmb;
    ledc_channel_cf.channel = m.channelb;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cf));
}

void set_motor_speed(motor_t m, int speed, bool forward)
{
    if (speed > 1023) speed = 1023; //sets max speed to 1023 for PMW
    
    if (forward)
    {
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channelb, speed));
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channela, 0));
    }
    else
    {
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channelb, 0));
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channela, speed));
    }
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, m.channela));
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, m.channelb));
}

void speed_callback(int32_t left_velocity, int32_t right_velocity)
{
    // Positive values = forward, negative values = backward
    set_motor_speed(left_motor, abs(left_velocity), left_velocity >= 0);
    set_motor_speed(right_motor, abs(right_velocity), right_velocity >= 0);
}
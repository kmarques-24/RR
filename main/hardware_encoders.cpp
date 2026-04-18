#include "include/hardware_encoders.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define TAG "ENCODER_SERVICE"

#define ENC_INTERVAL 5 // interval in ms to calculate count speed
#define ENC_MULTIPLIER (1000.0f / ENC_INTERVAL) // 1/(ENC_INTERVAL/1000) // Hz
#define ALPHA 0.2 // filtering

// use to temporarily lock so left & right vars can be updated together without interrupts
portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED; 

encoder_t left_encoder;
encoder_t right_encoder;

void init_encoder(encoder_t* encoder);

// ISR handler must not use non-ISR-safe functions like `gpio_get_level` unless GPIO is input-only and stable
void IRAM_ATTR encoder_isr_handler(void *arg)
{
    encoder_t *encoder = (encoder_t *)arg;

    int a_val = gpio_get_level(encoder->pin_a);
    int b_val = gpio_get_level(encoder->pin_b);
    int encoding = (a_val << 1) | b_val;

    int8_t increment = encoder->invert; 

    // Determine direction based on last state
    bool increment_count = (encoding == 0b00 && encoder->prevEncoding == 0b10) ||
                            (encoding == 0b11 && encoder->prevEncoding == 0b01);

    bool decrement_count = (encoding == 0b00 && encoder->prevEncoding == 0b01) ||
                            (encoding == 0b11 && encoder->prevEncoding == 0b10);

    // Read and increment together
    taskENTER_CRITICAL_ISR(&enc_mux);
    if (increment_count) encoder->count = encoder->count + increment;
    else if (decrement_count) encoder->count = encoder->count - increment;
    if (encoder->count >= CONFIG_CPR) encoder->count = 0; // wrap forward
    if (encoder->count < 0) encoder->count = CONFIG_CPR - 1; // wrap backward
    encoder->prevEncoding = encoding;
    taskEXIT_CRITICAL_ISR(&enc_mux);
}

void init_encoders(void)
{
    //static bool isr_service_installed = false;
    gpio_install_isr_service(0); //isr_service_installed);  // Only needs to be called once in app

    left_encoder.pin_a  = (gpio_num_t)CONFIG_LEFT_ENCODER_A;
    left_encoder.pin_b  = (gpio_num_t)CONFIG_LEFT_ENCODER_B;
    left_encoder.invert = 1;

    right_encoder.pin_a  = (gpio_num_t)CONFIG_RIGHT_ENCODER_A;
    right_encoder.pin_b  = (gpio_num_t)CONFIG_RIGHT_ENCODER_B;
    right_encoder.invert = 1; // -1 to invert. Don't need to due to gears

    init_encoder(&left_encoder);
    init_encoder(&right_encoder);
}

void init_encoder(encoder_t* encoder)
{
    ESP_LOGI(TAG, "Setting up pins %d and %d", encoder->pin_a, encoder->pin_b);

    // Configure input pins
    gpio_config_t io_conf= {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << encoder->pin_a) | (1ULL << encoder->pin_b);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // Initialize encoder state
    encoder->count = 0;
    encoder->prevCount = 0;
    encoder->countVelocity = 0.0f;
    encoder->prevEncoding = (gpio_get_level(encoder->pin_a) << 1) | gpio_get_level(encoder->pin_b);

    // Enable interrupt service - everything should be initialize before these lines
    gpio_isr_handler_add(encoder->pin_a, encoder_isr_handler, (void *)encoder); 
    gpio_isr_handler_add(encoder->pin_b, encoder_isr_handler, (void *)encoder);
}


void encoder_task(void *pvParameter)
{
   while(1)
   {
        // Uses set interval for velocity update
        // Velocity is signed, use for direction

        // Ensure counts are grabbed together without interrupt between
        taskENTER_CRITICAL(&enc_mux);
        int16_t countL = left_encoder.count;
        int16_t countR = right_encoder.count;
        taskEXIT_CRITICAL(&enc_mux);

        float oldSpeedL = left_encoder.countVelocity;
        int16_t countDiffLeft = differenceWrapped(countL, left_encoder.prevCount); // signed
        float newVelL = countDiffLeft * ENC_MULTIPLIER;
        left_encoder.prevCount = countL;
        left_encoder.countVelocity = newVelL * ALPHA + (1-ALPHA) * oldSpeedL;

        float oldSpeedR = right_encoder.countVelocity;
        int16_t countDiffRight = differenceWrapped(countR, right_encoder.prevCount); // signed
        float newVelR = countDiffRight * ENC_MULTIPLIER;
        right_encoder.prevCount = countR;
        right_encoder.countVelocity = newVelR * ALPHA + (1-ALPHA) * oldSpeedR;
    
        // ESP_LOGI("ENC", "Right rotations: %.2f", ((float)(right_encoder.count) / CPR));
        // ESP_LOGI("ENC", "Left rotations: %.2f", ((float)(left_encoder.count) / CPR));

        vTaskDelay(pdMS_TO_TICKS(ENC_INTERVAL));
   }
}

int16_t differenceWrapped(int16_t currentcount, int16_t prevcount)
{
    int16_t diff = currentcount - prevcount;

    // assumes wrap happened if difference travelled is over half a revolution
    if (diff > CONFIG_CPR / 2) diff -= CONFIG_CPR;  // took long way forward, adjust backward
    if (diff < -CONFIG_CPR / 2) diff += CONFIG_CPR;  // took long way backward, adjust forward

    return diff;  // positive = CW, negative = CCW
}


BaseType_t encoder_service(void)
{
   BaseType_t status = xTaskCreate(
       encoder_task,
       "encoder_task",
       4096,
       NULL,
       5,
       NULL);

   if (status == pdPASS)
   {
       ESP_LOGI(TAG, "Encoder service started");
   }
   else
   {
       ESP_LOGE(TAG, "Failed to start encoder service");
   }

   return status;
}
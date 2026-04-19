#include "include/hardware_encoders.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define TAG "ENCODER_SERVICE"

#define ENC_INTERVAL 10 // prev 5 // interval in ms to calculate count speed
#define ENC_MULTIPLIER (1000.0f / ENC_INTERVAL) // 1/(ENC_INTERVAL/1000) // Hz
#define ALPHA 0.2 // filtering

// use to temporarily lock so left & right vars can be updated together without interrupts
portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED; 

bool encoders_initialized = false;

encoder_t left_encoder;
encoder_t right_encoder;

esp_err_t init_encoder(encoder_t* encoder);

// commented prev -> current is the 4 bit lookup index
static const int8_t QUAD_TABLE[16] = {
     0,  // 00 -> 00  no change
    +1,  // 00 -> 01  CW
    -1,  // 00 -> 10  CCW
     0,  // 00 -> 11  illegal (both pins changed)
    -1,  // 01 -> 00  CCW
     0,  // 01 -> 01  no change
     0,  // 01 -> 10  illegal
    +1,  // 01 -> 11  CW
    +1,  // 10 -> 00  CW
     0,  // 10 -> 01  illegal
     0,  // 10 -> 10  no change
    -1,  // 10 -> 11  CCW
     0,  // 11 -> 00  illegal
    -1,  // 11 -> 01  CCW
    +1,  // 11 -> 10  CW
     0,  // 11 -> 11  no change
};

// ISR handler must not use non-ISR-safe functions like `gpio_get_level` unless GPIO is input-only and stable
void IRAM_ATTR encoder_isr_handler(void *arg)
{
    encoder_t *encoder = (encoder_t *)arg;

    int a_val = gpio_get_level(encoder->pin_a);
    int b_val = gpio_get_level(encoder->pin_b);
    int encoding = (a_val << 1) | b_val;

    taskENTER_CRITICAL_ISR(&enc_mux);

    // Prev & current together describe transition and make a unique 4-bit index
    int8_t delta = QUAD_TABLE[(encoder->prevEncoding << 2) | encoding];
    int16_t new_count = encoder->count + (delta * encoder->invert); // if motor flipped on bot

    // Wrap
    if (new_count >= CONFIG_CPR)     new_count -= CONFIG_CPR;
    else if (new_count < 0)          new_count += CONFIG_CPR;

    encoder->count = new_count;
    encoder->prevEncoding = encoding;

    taskEXIT_CRITICAL_ISR(&enc_mux);
}

void init_encoders(void)
{
    // Protects if already installed by another component
    esp_err_t isr_ret = gpio_install_isr_service(0); // Only needs to be called once in app
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install ISR service");
        return;
    }

    // -1 to invert and orient forward. May be different if motors are back
    // to back. Back to back but not different here due to gears (auto invert)
    left_encoder.pin_a  = (gpio_num_t)CONFIG_LEFT_ENCODER_A;
    left_encoder.pin_b  = (gpio_num_t)CONFIG_LEFT_ENCODER_B;
    left_encoder.invert = -1;

    right_encoder.pin_a  = (gpio_num_t)CONFIG_RIGHT_ENCODER_A;
    right_encoder.pin_b  = (gpio_num_t)CONFIG_RIGHT_ENCODER_B;
    right_encoder.invert = -1; 

    if (init_encoder(&left_encoder) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize left encoder");
        return;
    }
    if (init_encoder(&right_encoder) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize right encoder");
        return;
    }

    encoders_initialized = true;
}

esp_err_t init_encoder(encoder_t* encoder)
{
    ESP_LOGI(TAG, "Setting up pins %d and %d", encoder->pin_a, encoder->pin_b);

    // Configure input pins
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << encoder->pin_a) | (1ULL << encoder->pin_b);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // previously GPIO_PULLUP_ENABLE, but my circuit uses divider
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    if (gpio_config(&io_conf) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure GPIO pins %d and %d", encoder->pin_a, encoder->pin_b);
        return ESP_FAIL;
    }

    // Initialize encoder state
    encoder->count = 0;
    encoder->prevCount = 0;
    encoder->countVelocity = 0.0f;
    encoder->prevEncoding = (gpio_get_level(encoder->pin_a) << 1) | gpio_get_level(encoder->pin_b);

    // Enable interrupt service - everything should be initialized before these lines
    if (gpio_isr_handler_add(encoder->pin_a, encoder_isr_handler, (void *)encoder) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add ISR handler for pin %d", encoder->pin_a);
        return ESP_FAIL;
    }
    if (gpio_isr_handler_add(encoder->pin_b, encoder_isr_handler, (void *)encoder) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add ISR handler for pin %d", encoder->pin_b);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void encoder_task(void *pvParameter)
{
    // Log vars for debugging
    uint32_t log_counter = 0;
    const uint32_t LOG_INTERVAL_MS = 50; // log every 500ms
    const uint32_t LOG_EVERY_N = LOG_INTERVAL_MS / ENC_INTERVAL; // ticks between logs
   
    while(1)
    {
        // Uses set interval for velocity update
        // Velocity is signed, use sign for direction

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

        // Verify encoders work and velocity is positive in correct direction
        log_counter = log_counter + 1;
        if (log_counter >= LOG_EVERY_N)
        {
            // log_counter = 0;
            // ESP_LOGI(TAG, "L count: %d vel: %.1f prevEnc: %d%d | R count: %d vel: %.1f prevEnc: %d%d",
            //     countL, left_encoder.countVelocity,
            //     (left_encoder.prevEncoding >> 1) & 1, left_encoder.prevEncoding & 1,
            //     countR, right_encoder.countVelocity,
            //     (right_encoder.prevEncoding >> 1) & 1, right_encoder.prevEncoding & 1);
        }

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
    if (!encoders_initialized)
    {
        ESP_LOGE(TAG, "Encoders not initialized, cannot start service");
        return pdFAIL;
    }
   
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
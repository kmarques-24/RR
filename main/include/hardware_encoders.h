#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct encoder_t {
    volatile int16_t count;      
    volatile int16_t prevCount;
    volatile uint8_t prevEncoding;  
    volatile float countVelocity;   // counts/sec. sign CW=0, CCW=1 by convention
    gpio_num_t pin_a;
    gpio_num_t pin_b;
    int8_t invert;                  // for back-to-back motors
} encoder_t; // Variables that ISR writes to should be volatile

extern bool encoders_initialized;

extern portMUX_TYPE enc_mux;
extern encoder_t left_encoder;
extern encoder_t right_encoder; 

void init_encoders(void);
int16_t differenceWrapped(int16_t currentcount, int16_t prevcount);
void encoder_task(void *pvParameter);
BaseType_t encoder_service(void);

#ifdef __cplusplus
}
#endif
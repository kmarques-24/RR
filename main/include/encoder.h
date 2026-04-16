#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <fcntl.h>    // for open(), O_WRONLY, O_CREAT, etc.
#include <unistd.h>   // for close(), write(), etc.

typedef struct encoder_t {
    volatile int position;
    volatile int lastEncoding;
    gpio_num_t pin_a;
    gpio_num_t pin_b;
} encoder_t;

#define LEFT_ENCODER_A GPIO_NUM_40 
#define LEFT_ENCODER_B GPIO_NUM_41  

#define RIGHT_ENCODER_A GPIO_NUM_4 
#define RIGHT_ENCODER_B GPIO_NUM_42 
#define CPR (1375) //(Manually Counted for one full rotation for our motors. Change as needed) 


extern encoder_t left_encoder;
extern encoder_t right_encoder; 

void init_encoder(encoder_t* encoder);
void encoder_task(void);
BaseType_t encoder_service(void);

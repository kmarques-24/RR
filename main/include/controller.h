#pragma once

#include "freertos/FreeRTOS.h" // need for BaseType_t
#include "freertos/task.h"
#include <geometry_msgs/msg/twist.h>  

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float error;
    float prevError;
    float errorSum;
} motor_controller_t;

void drive_commanded_twist(const geometry_msgs__msg__Twist *twist_msg);
void init_controller(void);
BaseType_t start_controller(void);

#ifdef __cplusplus
}
#endif
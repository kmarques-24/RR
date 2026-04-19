#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "utils.h"
#include <nav_msgs/msg/odometry.h>   

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct {float x, y, z;} position;       // Position
    struct {float w, x, y, z;} quat;        // Orientation (quat)

    struct {float x, y, z;} twist_lin;      // Velocity (linear)
    struct {float x, y, z;} twist_ang;      // Velocity (angular)

    timespec_t timestamp;
} odom_data_t;

extern bool estimator_initialized; // TODO: Check this elsewhere before dependent code runs

void update_odometry_msg(nav_msgs__msg__Odometry *odom_msg);
void init_estimator(void);

BaseType_t start_estimator(void);

#ifdef __cplusplus
}
#endif
#pragma once

#include "freertos/FreeRTOS.h" // must always include before task.h
#include "freertos/task.h"
#include <sensor_msgs/msg/imu.h>  

#ifdef __cplusplus
extern "C" {
#endif

void init_imu();

extern TaskHandle_t estimator_task_handle;

typedef struct {
    struct {float w, x, y, z;} quat;        // Orientation (quat)
    struct {float roll, pitch, yaw;} euler; // Orientation (euler)
    struct {float x, y, z;} lin_accel;      // Acceleration (linear)
    struct {float x, y, z;} ang_vel;        // Velocity (angular)
    struct {float x, y, z;} mag;            // Magnetometer
    uint32_t timestamp;
} imu_data_t;

void update_imu_msg(sensor_msgs__msg__Imu *imu_msg);
BaseType_t imu_service(void);

#ifdef __cplusplus
}
#endif

#pragma once

#include <nav_msgs/msg/odometry.h>   

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t start_estimator(void);

void update_odometry_msg(nav_msgs__msg__Odometry *odom_msg);

typedef struct {
    imu_data_t imu_data;
    // TODO: encoder data for position, velocity, heading
} bot_state_t;

#ifdef __cplusplus
}
#endif
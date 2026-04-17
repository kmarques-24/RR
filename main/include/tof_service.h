#pragma once

// Header includes
#include "vl53l5cx_api.h"
#include "utils.h"
#include <sensor_msgs/msg/point_cloud2.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    VL53L5CX_ResultsData results;
    timespec_t timestamp;        // system timestamp
} tof_data_t;

void init_tof_sensor(void);
void update_tof_msg(sensor_msgs__msg__PointCloud2 *tof_msg);

#ifdef __cplusplus
}
#endif
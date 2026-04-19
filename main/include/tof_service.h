#pragma once

// Header includes
#include "vl53l5cx_api.h"
#include "utils.h"
#include <sensor_msgs/msg/point_cloud2.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOF_FOV_DEG     48.5f   // per-axis FoV derived from 65° diagonal for square 8x8 array
#define TOF_CELL_ANGLE_RAD  (TOF_FOV_DEG * M_PI / 180.0f / TOF_GRID_SIZE)  // ~0.1064 rad per cell
#define TOF_GRID_SIZE   8                                  // cells per side
#define TOF_GRID_CENTER 3.5f                               // (TOF_GRID_SIZE / 2) - 0.5
#define TOF_NUM_ZONES   (TOF_GRID_SIZE * TOF_GRID_SIZE)    // 64
#define TOF_FIELDS      3                                  // x, y, z
#define TOF_BYTES_PER_FIELD  4                             // float32 = 4 bytes
#define TOF_POINT_STEP  (TOF_FIELDS * TOF_BYTES_PER_FIELD) // 12 bytes per point
#define TOF_BUF_SIZE    (TOF_NUM_ZONES * TOF_POINT_STEP)   // 768 bytes
/*
Cell indices:  0    1    2    3  |  4    5    6    7
                            center = 3.5
Offsets:     -3.5 -2.5 -1.5 -0.5  +0.5 +1.5 +2.5 +3.5
*/

typedef struct {
    VL53L5CX_ResultsData results;
    timespec_t timestamp;        // system timestamp
} tof_data_t;

void init_tof_sensor(void);
void update_tof_msg(sensor_msgs__msg__PointCloud2 *tof_msg);

extern bool tof_initialized; // TODO: Check this elsewhere before dependent code runs

uint8_t start_ranging(void);
uint8_t stop_ranging(void);
BaseType_t tof_service(void);

#ifdef __cplusplus
}
#endif
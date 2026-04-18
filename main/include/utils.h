#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    int32_t secs;
    uint32_t nanosecs;
} timespec_t;

timespec_t getTime(void);

#define dx 0.2
#define dy 0.4

float compute_omega(float x, float y, float v);         // Compute angular velocity from x, y, and v
float compute_v(float x, float y, float v, float w);    // Compute center of mass velocity from x, y, v, and w

void cross_product(float a[3], float b[3], float c[3]); // Compute cross product of a and b and store in c
void normalize(float a[3]);                             // Normalize vector a
void subtract(float a[3], float b[3], float c[3]);      // Subtract b from a and store in c

#ifdef __cplusplus
}
#endif
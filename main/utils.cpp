#include "utils.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    #include <rmw_microros/rmw_microros.h>
#endif

timespec_t getTime(void)
{
    timespec_t time;
    int64_t time_ns = rmw_uros_epoch_nanos(); // uses synced clock
    time.secs     = (int32_t)(time_ns / 1000000000);
    time.nanosecs = (uint32_t)(time_ns % 1000000000);
    return time;
}

float compute_omega(float x, float y, float v)
{
    float omega = 0.0;
    float r = sqrt(x * x + y * y);
    if (x > 0)
    {
        omega = -v / r;
    }
    else if (x < 0)
    {
        omega = v / r;
    }
    return omega;
}
float compute_v(float x, float y, float v, float w)
{
    float r_vec[] = {x, y, 0};
    float v_vec[] = {v, 0, 0};
    float w_vec[] = {0, 0, w};
    float w_cross_r[3];
    cross_product(w_vec, r_vec, w_cross_r);
    float v_new[3];
    subtract(v_vec, w_cross_r, v_new);
    return v_new[0]; // Return x component of v_new, both y and z components should be 0
}

void cross_product(float a[3], float b[3], float c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

void normalize(float a[3])
{
    float norm = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    a[0] = a[0] / norm;
    a[1] = a[1] / norm;
    a[2] = a[2] / norm;
}

void subtract(float a[3], float b[3], float c[3])
{
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}
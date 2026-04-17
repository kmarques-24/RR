#include "helpers.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

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
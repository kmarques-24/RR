#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h> 
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "uros_service.h"

static const char *TAG = "uros_service";

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg; // double underscore: C-naming convention for ROS 2 messages

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        printf("Publishing: %f\n", msg.data); // where actual publishing happens
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

// Based on int32 publisher example
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        // Static Agent IP and port can be used instead of autodisvery.
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    #endif

    // create init options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "Rescue Roller Node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), // helps ROS 2 agent decode bits
        "float32publisher"));
    
    // create timer
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000; // timer_callback triggered once every second
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    
    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // check for work and wait up to 100 ms
        usleep(10000); // pauses task for 10 ms (usleep units are microseconds, so 10000 us)

        // rclc_executor is a checker. spin_some waits for a ROS event and checks out after 100 ms
        // then it microsleeps if someone else has a task before restarting
        // rclc_executor_spin blocks forever and loop never repeats. Use if task only handles ROS events
        // rclc_executor_prepare and rclc_executor_spin_one_period can help sync ROS with non-ROS control loop
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

// function
// BaseType_t is flexible data type that can map int based on platform
// here it's a 32-bit signed integer
BaseType_t uros_service(void)
{
    BaseType_t status;
    status = xTaskCreate(
        micro_ros_task,
        "uros_task",
        8192,   // Stack size
        NULL,
        5,      // Priority
        NULL);

    if (status == pdPASS)
    {
        ESP_LOGI(TAG, "service started");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting the service");
    }
    return status;
}

/*
void app_main(void)
{
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL,
        1  // Core 1 (APP_CPU)
    );
    

    //lets espidf scheduler decide which core to use
    //code originally from example with intention to pin per comment above
    
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
    

    The ESP32 is dual-core:
    PRO_CPU (Core 0): Usually handles the "Protocol" (Wi-Fi and Bluetooth stacks).
    APP_CPU (Core 1): Usually handles your "Application" logic.
}
*/
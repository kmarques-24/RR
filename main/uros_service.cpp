#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"
#include "esp_event.h"

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
    ESP_LOGI(TAG, "Started uros service. Status: RUNNING (Connected to WiFi)");
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

        // adjust retransmission and timeout settings to wait for slow netrwork
        rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options); // Set a static key to stop the 'cycling'

        // Static Agent IP and port can be used instead of autodiscovery - what we have below
        // IP address and port on my computer autodetermined at 192.168.1.100 and 8888 (sdkconfig)
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    #endif

    // create init options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    // Replace the init line with a retry loop
    /*
    while (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
        ESP_LOGW(TAG, "Waiting for Agent at %s:%s...", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    */

    // check for laptop to respond
    /*
    ESP_LOGI(TAG, "Pinging Agent to verify handshake...");
    while (rmw_uros_ping_agent(500, 5) != RCL_RET_OK) {
        ESP_LOGE(TAG, "Agent unreachable via RMW Ping. Waiting for stable connection...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Agent is ALIVE. Proceeding to node creation.");
    */

    // create node - name can't have spaces
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "Rescue_Roller_Node", "", &support));

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

// Function returning BaseType_t, a flexible data type that can map int based on platform
//      here it's a 32-bit signed integer
BaseType_t uros_service(void)
{   
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    BaseType_t status;
    status = xTaskCreate(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK, // 8192, Stack size
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO, // 5,  Priority
        NULL);

    if (status != pdPASS)
    {
        ESP_LOGI(TAG, "Error starting uros service.");
    }
    return status;
}
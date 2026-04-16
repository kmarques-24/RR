// MicroROS service to publish to laptop agent

/* ------ INCLUDES ------ */
// Standard includes
#include <string.h>
#include <stdio.h>
#include <unistd.h>

// Header includes
#include "uros_service.h"
#include "estimator.h"
#include "tof_service.h"

// FreeRTOS and ESP32 includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"
#include "esp_event.h"

// microROS includes
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>       
#include <sensor_msgs/msg/range.h>      
#include <nav_msgs/msg/odometry.h>      
#include <geometry_msgs/msg/twist.h>    
#include <rclc/rclc.h>
#include <rclc/executor.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif


/* ------ GLOBALS ------ */
// Publishers & subscribers
rcl_publisher_t float_pub;      // float for testing: originally based on int32 publisher example
rcl_publisher_t tof_pub;        // ToF sensor
rcl_publisher_t odom_pub;       // RR state (odometry)
rcl_subscription_t twist_sub;   // Get linear & angular velocity commands. E.g. from keyboard

// Messages
std_msgs__msg__Float32 float_msg;   // float msg. double underscore is C-naming convention for ROS 2 messages
sensor_msgs__msg__Range range_msg;  // ToF msg
nav_msgs__msg__Odometry odom_msg;   // RR state (odometry) msg
geometry_msgs__msg__Twist twist_msg;    // twist command msg

// Timers
rcl_timer_t float_timer;
rcl_timer_t tof_timer;
rcl_timer_t odom_timer;

// Tag to identify log statement source
static const char *TAG = "uros_service";


/* ------ CALLBACKS ------ */
void float_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        printf("Publishing: %f\n", float_msg.data); // debug statement to read via idf.py monitor
        RCSOFTCHECK(rcl_publish(&float_pub, &float_msg, NULL)); // where actual publishing happens
        float_msg.data++;
    }
}

void tof_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        range_msg.range = read_tof_sensor();
        RCSOFTCHECK(rcl_publish(&tof_pub, &range_msg, NULL));
        float_msg.data++;
    }
}

void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        update_odometry_msg(&odom_msg); // pass as pointer to modify multiple fields
        RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
        float_msg.data++;
    }
}


/* ------ MICRO-ROS TASK ------ */
void micro_ros_task(void *arg)
{   
    // Setup
    ESP_LOGI(TAG, "Started uros service. Status: RUNNING (Connected to WiFi)");
    
    rcl_allocator_t allocator = rcl_get_default_allocator(); // how uros should manage esp32 RAM
    rclc_support_t support; // connection state and system configs

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options(); // settings to establish connections
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options); 
        // Set a static key to stop key cycling
        rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options); 
        // Set in menuconfig after connecting laptop to router and checking IP
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    #endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)); // open comm line with laptop

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "Rescue_Roller_Node", "", &support)); // create node - name can't have spaces

    // Create publishers
        // switched from rclc_publisher_init_default to rclc_publisher_init_best_effort
        // best effort is better for continuous data streams where it's ok for a packet to drop in
        // order to keep stream flowing. Otherwise, buffer clogs trying to save failed packets
    RCCHECK(rclc_publisher_init_best_effort(
        &float_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), // helps ROS 2 agent decode bits
        "float32publisher"));
    
    // Create timers
    const unsigned int timer_timeout = 1000; // timer_callback triggered once every second
    RCCHECK(rclc_timer_init_default(
        &float_timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        float_timer_callback));
    
    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &float_timer));

    // Initialize message data
    float_msg.data = 0;
    
    // Spin
    while (1)
    {   
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // check for work and wait up to 100 ms
        usleep(10000); // pauses task for 10 ms (usleep units are microseconds, so 10000 us)

        // rclc_executor is a checker. spin_some waits for a ROS event and checks out after 100 ms
            // then it microsleeps if someone else has a task before restarting
            // rclc_executor_spin blocks forever and loop never repeats. Use if task only handles ROS events
            // rclc_executor_prepare and rclc_executor_spin_one_period can help sync ROS with non-ROS control loop
    }

    // Free resources
    RCCHECK(rcl_publisher_fini(&float_pub, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}


/* ------ CREATE AND START TASK ------ */
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
    return status;  // returns BaseType_t, a flexible data type that can match int based on platform
                    // here it's a 32-bit signed integer
}
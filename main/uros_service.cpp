// MicroROS service to publish to laptop agent

/* ------ INCLUDES ------ */
#include "uros_service.h"

// Header includes
#include "estimator.h"
#include "tof_service.h"
#include "imu_service.h"
#include "controller.h"
#include "rr_os_service.h"

// Standard includes
#include <string.h>
#include <stdio.h>
#include <unistd.h>

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
#include <rclc/rclc.h>
#include <rclc/executor.h>

// message types includes
#include <std_msgs/msg/float32.h>       
#include <sensor_msgs/msg/point_cloud2.h>  
#include <sensor_msgs/msg/point_field.h> // component of point cloud   
#include <nav_msgs/msg/odometry.h>   
#include <sensor_msgs/msg/imu.h>   
#include <geometry_msgs/msg/twist.h>    

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif


/* ------ GLOBALS ------ */
// Publishers & subscribers
rcl_publisher_t float_pub;      // float for testing: originally based on int32 publisher example
rcl_publisher_t tof_pub;        // ToF sensor
rcl_publisher_t odom_pub;       // RR state (odometry)
rcl_publisher_t imu_pub;        // IMU
rcl_subscription_t twist_sub;   // Get linear & angular velocity commands. E.g. from keyboard

// Messages
std_msgs__msg__Float32 float_msg;           // float msg. double underscore is C-naming convention for ROS 2 messages
sensor_msgs__msg__PointCloud2 tof_msg;    // ToF msg
nav_msgs__msg__Odometry odom_msg;           // RR state (odometry) msg
sensor_msgs__msg__Imu imu_msg;              // IMU msg
geometry_msgs__msg__Twist twist_msg;             // twist msg

// Timers
rcl_timer_t float_timer;
rcl_timer_t tof_timer;
rcl_timer_t odom_timer;
rcl_timer_t imu_timer;

// Tag to identify log statement source
static const char *TAG = "uros_service";

void initialize_message_data(void);


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
        update_tof_msg(&tof_msg);
        RCSOFTCHECK(rcl_publish(&tof_pub, &tof_msg, NULL));
    }
}

void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        update_odometry_msg(&odom_msg); // pass as pointer to modify multiple fields
        RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
    }
}

void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // "uses" the variable to suppressed unused variable complier warning
    // guards against NULL timer
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        update_imu_msg(&imu_msg);
        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
    }
}

void twist_callback(const void *msgin)
{
    if (msgin == NULL) return; // just in case
    
    // pointer-to-const ensures incoming message not modified
    // can't edit target (data). Just a subscriber. 
    // Executor owns original twist_msg and can overwrite
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    drive_commanded_twist(msg); // pass protected msg, not twist_msg
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

    // Make sure IP is right if failing here
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)); // open comm line with laptop

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "Rescue_Roller_Node", "", &support)); // create node - name can't have spaces

    // Create publishers and subscriber
        // switched from rclc_publisher_init_default to rclc_publisher_init_best_effort
        // best effort is better for continuous data streams where it's ok for a packet to drop in
        // order to keep stream flowing. Otherwise, buffer clogs trying to save failed packets
        // topic names standard for ROS2 packages
    RCCHECK(rclc_publisher_init_best_effort(&float_pub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "float_debug")); // topic name
    if (rr_status.tof_enabled) 
    {
        RCCHECK(rclc_publisher_init_best_effort(&tof_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2), "points"));
    }
    if (rr_status.estimator_enabled) 
    {
        RCCHECK(rclc_publisher_init_best_effort(&odom_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
    }
    if (rr_status.imu_enabled) 
    {
        RCCHECK(rclc_publisher_init_best_effort(&imu_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));
    }
    if (rr_status.key_control_enabled) 
    {
        RCCHECK(rclc_subscription_init_best_effort(&twist_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    }

    // Create executor
    rclc_executor_t executor;
    // Count how many handles the executor needs
    int executor_handles = 1; // float timer always active
    if (rr_status.tof_enabled)          executor_handles++;
    if (rr_status.estimator_enabled)    executor_handles++;
    if (rr_status.imu_enabled)          executor_handles++;
    if (rr_status.key_control_enabled)  executor_handles++;
    RCCHECK(rclc_executor_init(&executor, &support.context, executor_handles, &allocator));

    // Create timers and add to executor
    const unsigned int float_timeout = 1000; // once per sec = 1 Hz
    const unsigned int tof_timeout = 50; // 20 Hz
    const unsigned int odom_timeout = 20; // 50 Hz
    const unsigned int imu_timeout = 10; // 100 Hz

    RCCHECK(rclc_timer_init_default(&float_timer, &support,
        RCL_MS_TO_NS(float_timeout), float_timer_callback));
    RCCHECK(rclc_executor_add_timer(&executor, &float_timer));

    if (rr_status.tof_enabled) 
    {
        RCCHECK(rclc_timer_init_default(&tof_timer, &support,
            RCL_MS_TO_NS(tof_timeout), tof_timer_callback));
        RCCHECK(rclc_executor_add_timer(&executor, &tof_timer));
    }
    if (rr_status.estimator_enabled) 
    {
        RCCHECK(rclc_timer_init_default(&odom_timer, &support,
            RCL_MS_TO_NS(odom_timeout), odom_timer_callback));
        RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
    }
    if (rr_status.imu_enabled) 
    {
        RCCHECK(rclc_timer_init_default(&imu_timer, &support,
            RCL_MS_TO_NS(imu_timeout), imu_timer_callback));
        RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
    }
    if (rr_status.key_control_enabled) 
    {
        RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub,
            &twist_msg, &twist_callback, ON_NEW_DATA));
    }

    // Initialize published message data
    initialize_message_data();

    //TickType_t xLastWakeTime = xTaskGetTickCount(); // vTaskDelayUntil?
    //const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 Hz (50 ms)

    // Sync with microros agent on laptop before spinning. Align timestamps
    rmw_uros_sync_session(1000);
    
    // Spin
    while (1)
    {   
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)); // check for work and wait up to 5 ms
            // 5 ms must be shorter than fastest timer period
        vTaskDelay(pdMS_TO_TICKS(1)); // wait 1 ms
            //usleep(10000); // pauses task for 10 ms (usleep units are microseconds, so 10000 us)

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

void initialize_message_data(void)
{
    // fields auto intialized to 0 unless changed. Some 0s listed for visible redundancy

    //----- float -----
    float_msg.data = 0;
    
    //----- tof -----
    static char frame_id_tof[] = "tof_link";
    static char field_name_x[] = "x";
    static char field_name_y[] = "y";
    static char field_name_z[] = "z";

    tof_msg.header.frame_id.data = frame_id_tof; // label that points are expressed in tof coordinate frame
    tof_msg.header.frame_id.size = strlen("tof_link"); // "_link" naming convention: rigid body frame
    tof_msg.header.frame_id.capacity = strlen("tof_link") + 1; // +1 for null terminator
    tof_msg.header.stamp.sec = 0;
    tof_msg.header.stamp.nanosec = 0;

    static sensor_msgs__msg__PointField field_arr[TOF_FIELDS]; // 3 for x, y, z
    // point is endpoint of ray relative to sensor. ROS convention is z forward, x right, y up
    // pointfield stores column definition like in table where each row is a point
    // float32 is 4 bytes. So a row of x, y, z will have x at byte 0, y at byte 4, z at byte 8
    field_arr[0].name.data = field_name_x; field_arr[0].name.size = strlen("x"); 
    field_arr[0].offset = 0 * TOF_BYTES_PER_FIELD;  
    field_arr[0].datatype = sensor_msgs__msg__PointField__FLOAT32; 
    field_arr[0].count = 1;

    field_arr[1].name.data = field_name_y; 
    field_arr[1].name.size = strlen("y"); 
    field_arr[1].offset = 1 * TOF_BYTES_PER_FIELD;  
    field_arr[1].datatype = sensor_msgs__msg__PointField__FLOAT32; 
    field_arr[1].count = 1;

    field_arr[2].name.data = field_name_z; 
    field_arr[2].name.size = strlen("z");
    field_arr[2].offset = 2 * TOF_BYTES_PER_FIELD;  
    field_arr[2].datatype = sensor_msgs__msg__PointField__FLOAT32;
    field_arr[2].count = 1;

    tof_msg.fields.data = field_arr;
    tof_msg.fields.size = TOF_FIELDS;
    tof_msg.fields.capacity = TOF_FIELDS;

    static uint8_t point_buf[TOF_BUF_SIZE]; // 64 points, 12 bytes each (4 bytes per float * 3 floats xyz)
    tof_msg.data.data = point_buf;
    tof_msg.data.size = TOF_BUF_SIZE;
    tof_msg.data.capacity = TOF_BUF_SIZE;

    tof_msg.height = TOF_GRID_SIZE; // preserve grid structure. Could also do height=1 width=64 for flat list
    tof_msg.width = TOF_GRID_SIZE;
    tof_msg.is_bigendian = false; //esp32 little endian
    tof_msg.is_dense = true;
    tof_msg.point_step = TOF_POINT_STEP; // byte size of one point (4 bytes per float * 3 floats xyz)
    tof_msg.row_step = TOF_POINT_STEP * TOF_GRID_SIZE;

    //----- odom -----
    static char frame_id_odom[] = "odom";
    static char child_frame_id[] = "base_link";

    odom_msg.header.frame_id.data = frame_id_odom; // world frame, defined when bot first powered on
    odom_msg.header.frame_id.size = strlen("odom");
    odom_msg.header.frame_id.capacity = strlen("odom") + 1;
    odom_msg.header.stamp.sec = 0;
    odom_msg.header.stamp.nanosec = 0;
    
    odom_msg.child_frame_id.data = child_frame_id;         // body frame (center of rotation of robot frame) 
    odom_msg.child_frame_id.size = strlen("base_link"); // want to know body frame expressed in world frame
    odom_msg.child_frame_id.capacity = strlen("base_link") + 1;

    // position of body in world in m
    odom_msg.pose.pose.position.x = 0; 
    odom_msg.pose.pose.position.y = 0;
    // z is 0 for now on flat surface testing - TODO incorporate

    // rotation of body in world as quat
    odom_msg.pose.pose.orientation.w = 1;   // init facing forward, no rotation
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;

    // velocity of body in m/s
    odom_msg.twist.twist.linear.x = 0; // only x matters for diff drive robot whose options are forward/backward

    // angular velocity of body in rad/s
    odom_msg.twist.twist.angular.z = 0;  // yaw rate
    // roll & pitch 0 for now on flat surface testing - TODO incorporate

    // Diagonal
    // [0]  = x variance
    // [7]  = y variance
    // [14] = z variance
    // [21] = roll variance
    // [28] = pitch variance
    // [35] = yaw variance

    // guesstimate initial covariances (uncertainties)
    odom_msg.pose.covariance[0] = 0.001;     // x
    odom_msg.pose.covariance[7] = 0.001;     // y
    odom_msg.pose.covariance[35] = 0.005;    // yaw

    odom_msg.twist.covariance[0] = 0.001;    // x rate (velocity uncertainty from encoder calc)
    odom_msg.twist.covariance[35] = 0.002;   // yaw rate (uncertainty from IMU)

    //----- imu -----
    static char frame_id_imu[] = "imu_link";

    imu_msg.header.frame_id.data = frame_id_imu;
    imu_msg.header.frame_id.size = strlen("imu_link");
    imu_msg.header.frame_id.capacity = strlen("imu_link") + 1;
    imu_msg.header.stamp.sec = 0;
    imu_msg.header.stamp.nanosec = 0;

    imu_msg.orientation.w = 0;
    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation_covariance[0] = 0.004; // roll
    imu_msg.orientation_covariance[4] = 0.004; // pitch
    imu_msg.orientation_covariance[8] = 0.004; // yaw

    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 0;
    imu_msg.angular_velocity_covariance[0] = 0.003; // x rate
    imu_msg.angular_velocity_covariance[4] = 0.003; // y rate
    imu_msg.angular_velocity_covariance[8] = 0.003; // z rate

    imu_msg.linear_acceleration.x = 0;
    imu_msg.linear_acceleration.y = 0;
    imu_msg.linear_acceleration.z = 0;
    imu_msg.linear_acceleration_covariance[0] = 0.09; // x acc
    imu_msg.linear_acceleration_covariance[0] = 0.09; // y acc
    imu_msg.linear_acceleration_covariance[0] = 0.09; // z acc
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
        8192, // 8192, Stack size
        NULL,
        5, // 5,  Priority
        NULL);

    if (status != pdPASS)
    {
        ESP_LOGI(TAG, "Error starting uros service.");
    }
    return status;  // returns BaseType_t, a flexible data type that can match int based on platform
                    // here it's a 32-bit signed integer
}
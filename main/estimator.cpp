#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"
#include "esp_log.h"
#include "esp_event.h"
#include "imu_service.h"
#include "estimator.h" // can't go at top for some reason

static const char *TAG = "Estimator";

void update_odometry_msg(nav_msgs__msg__Odometry *odom_msg)
{
    // (*p).field = p->field
    odom_msg->header.stamp.sec = 0;
    odom_msg->header.stamp.nanosec = 0;

    odom_msg->pose.covariance[0] = 0;
    odom_msg->pose.pose.position.x = 0;
    odom_msg->pose.pose.position.y = 0;
    odom_msg->pose.pose.position.z = 0;
    odom_msg->pose.pose.orientation.w = 0; // quaternion
    odom_msg->pose.pose.orientation.x = 0;
    odom_msg->pose.pose.orientation.y = 0;
    odom_msg->pose.pose.orientation.z = 0;

    odom_msg->twist.covariance[0] = 0;
    odom_msg->twist.twist.angular.x = 0;
    odom_msg->twist.twist.angular.y = 0;
    odom_msg->twist.twist.angular.z = 0;
    odom_msg->twist.twist.linear.x = 0;
    odom_msg->twist.twist.linear.y = 0;
    odom_msg->twist.twist.linear.z = 0;
}

void init_estimator(void)
{
    ESP_LOGI(TAG, "Estimator enabled");

    // TODO: init state
    
}

void estimator_task(void *pvParameter)
{
    imu_data_t imu_data;
    
    while(1)
    {
        // wait for notification from IMU task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_data = get_imu_latest();

        // TODO: populate state
    }
}

BaseType_t start_estimator(void)
{
    BaseType_t status;
    status = xTaskCreate(
        estimator_task,
        "estimator_task",
        4096,
        NULL,
        6,
        &estimator_task_handle);
    
    if (status == pdPASS)
    {
        ESP_LOGI(TAG, "Estimator started!");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting estimator.");
    }
    return status;
}
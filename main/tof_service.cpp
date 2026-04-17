#include "tof_service.h"
#include "utils.h"

// FreeRTOS and ESP32 includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "esp_timer.h"

#include <sensor_msgs/msg/point_cloud2.h>

// Standard includes
#include <string.h>
#include <stdio.h>
#include <unistd.h>

static const char *TAG = "ToF";

//VL53L5CX ranging variables
static uint8_t 				status, loop, isAlive, isReady, i;
static VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
static VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */

static SemaphoreHandle_t dataMutex; // protect while writing
static StaticSemaphore_t dataMutexBuffer;

// Global instance
static tof_data_t tof_latest;

void update_tof_msg(sensor_msgs__msg__PointCloud2 *tof_msg)
{
    // lock here too to avoid partial data read
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    // TODO: match tof data to tof msg
    tof_msg->header.stamp.sec     = tof_latest.timestamp.secs;
    tof_msg->header.stamp.nanosec = tof_latest.timestamp.nanosecs;

    /*
    // For cell at row r, col c in an 8x8 grid with 45° FoV:
    float fov = 45.0f * M_PI / 180.0f;           // total FoV in radians
    float cell_angle = fov / 8.0f;                // angle per cell

    float theta_x = (c - 3.5f) * cell_angle;     // horizontal angle from center
    float theta_y = (r - 3.5f) * cell_angle;     // vertical angle from center
    float d = distances[r][c] / 1000.0f;         // mm to meters

    // ROS convention: z forward, x right, y up
    float z = d * cosf(theta_x) * cosf(theta_y); // depth (forward)
    float x = d * sinf(theta_x);                 // horizontal
    float y = d * sinf(theta_y);                 // vertical
    */
    
    xSemaphoreGive(dataMutex);
}

void init_tof_sensor(void)
{
    esp_err_t status;
    uint8_t isAlive;

    // 1. Define the Legacy I2C configuration
    Dev.platform.port = I2C_NUM_1;
    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1; // 7-bit address
    Dev.platform.reset_gpio = GPIO_NUM_5;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 1,           // SDA GPIO
        .scl_io_num = 2,           // SCL GPIO
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = VL53L5CX_MAX_CLK_SPEED,
        },
    };

    // 2. Configure and Install the driver
    ESP_ERROR_CHECK(i2c_param_config(Dev.platform.port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(Dev.platform.port, conf.mode, 0, 0, 0));

    /* 3. (Optional) Reset sensor using your new platform function */
    VL53L5CX_Reset_Sensor(&(Dev.platform));

    /* 4. Check if sensor is alive */
    status = vl53l5cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status)
    {
        ESP_LOGE("TOF", "VL53L5CX not detected at 0x%02X", Dev.platform.address);
        return;
    }

    /* 5. (Mandatory) Init VL53L5CX sensor */
    status = vl53l5cx_init(&Dev);
    if(status)
    {
        ESP_LOGE("TOF", "VL53L5CX ULD Loading failed, status: %u", status);
        return;
    }

    ESP_LOGI("TOF", "VL53L5CX ULD ready! (Version: %s)", VL53L5CX_API_REVISION);

    // mutex
    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
}

void start_ranging(void)
{
    status = vl53l5cx_start_ranging(&Dev);
}

void stop_ranging(void)
{
    status = vl53l5cx_stop_ranging(&Dev);
}

void tof_task(void *pvParameter)
{
    // see example in components/VL53L5CX-Library/examples/ranging_basic/main/main.c
    while(1)
    {
        status = vl53l5cx_check_data_ready(&Dev, &isReady);

        if(isReady)
        {
            //uint32_t tof_time_ms = (uint32_t)((esp_timer_get_time() / 1000));
            timespec_t time = getTime();

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            vl53l5cx_get_ranging_data(&Dev, &Results);
            tof_latest.results = Results;
            tof_latest.timestamp = time;
            xSemaphoreGive(dataMutex);
        }

        /* Wait a few ms to avoid too high polling (function in platform file, not in API) */
        VL53L5CX_WaitMs(&(Dev.platform), 20); // calles vTaskDelay under hood
        // 20 ms is 50 Hz. Changed from 5 ms
    }
}

BaseType_t tof_service(void)
{
    BaseType_t tof_status;
    tof_status = xTaskCreate(
        tof_task,
        "tof_task",
        8192,
        NULL,
        4,
        NULL);
    
    if (tof_status == pdPASS)
    {
        ESP_LOGI(TAG, "ToF service started!");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting ToF service.");
    }
    return tof_status;
}
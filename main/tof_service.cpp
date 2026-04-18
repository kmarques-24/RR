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
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

static const char *TAG = "ToF";

//VL53L5CX ranging variables
static uint8_t status, isAlive;
static VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
static VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */

static SemaphoreHandle_t dataMutex; // protect while writing
static StaticSemaphore_t dataMutexBuffer;

bool tof_initialized = false;

// Global instance
static tof_data_t tof_latest;

void update_tof_msg(sensor_msgs__msg__PointCloud2 *tof_msg)
{
    // lock here too to avoid partial data read
    xSemaphoreTake(dataMutex, portMAX_DELAY);

    tof_msg->header.stamp.sec     = tof_latest.timestamp.secs;
    tof_msg->header.stamp.nanosec = tof_latest.timestamp.nanosecs;

    // For cell at row r, col c in an 8x8 grid
    // 65 deg diagonal field of view per datasheet -> eq to get vertical/horizontal -> 48.5 deg
    // https://discussions.unity.com/t/calculate-diagonal-field-of-view/772042/4
    // search "camera diagonal FOV from horizontal vertical FOV", rearrange & solve

    uint8_t *ptr = tof_msg->data.data; // rest of function just updates data

    for (int r = 0; r < TOF_GRID_SIZE; r++) {
        for (int c = 0; c < TOF_GRID_SIZE; c++) {
            int i = r * TOF_GRID_SIZE + c;

            // d is length of ray and hypotenuse of triangle
            float d = tof_latest.results.distance_mm[i] / 1000.0f;  // mm to m
            float theta_x = (c - TOF_GRID_CENTER) * TOF_CELL_ANGLE_RAD; // horizontal angle from center in rad
            float theta_y = (r - TOF_GRID_CENTER) * TOF_CELL_ANGLE_RAD; // vertical angle from center in rad

            // ROS convention: z forward, x right, y up
            float z = d * cosf(theta_x) * cosf(theta_y); // depth (forward)
            float x = d * sinf(theta_x); // horizontal
            float y = d * sinf(theta_y); // vertical

            memcpy(ptr + 0 * TOF_BYTES_PER_FIELD, &x, TOF_BYTES_PER_FIELD);
            memcpy(ptr + 1 * TOF_BYTES_PER_FIELD, &y, TOF_BYTES_PER_FIELD);
            memcpy(ptr + 2 * TOF_BYTES_PER_FIELD, &z, TOF_BYTES_PER_FIELD);

            ptr += TOF_POINT_STEP;
        }
    }
    xSemaphoreGive(dataMutex);
}

void init_tof_sensor(void)
{
    // ----- I2C setup (legacy I2C config) -----
    Dev.platform.port = I2C_NUM_1;
    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1; // 7-bit address
    Dev.platform.reset_gpio = GPIO_NUM_NC; // reset pin not connected

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)CONFIG_TOF_SDA, // SDA GPIO
        .scl_io_num = (gpio_num_t)CONFIG_TOF_SCL, // SCL GPIO
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = VL53L5CX_MAX_CLK_SPEED,
        },
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(Dev.platform.port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(Dev.platform.port, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C params configured and driver installed");

    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) // for all valid 7-bit i2c addresses:
    {
        // made command buffer - linked list of i2c operations to execute in sequence   
        i2c_cmd_handle_t cmd = i2c_cmd_link_create(); 
        i2c_master_start(cmd); // queue start condition to begin i2c transaction
        // write one byte on port (address + write bit 0) and expect acknowledgement (true)
        // if device exists at address, SDA will be pulled low
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd); // stop transaction
        // executes queued commands - retun ok if device ACKed (acknowledged)
        esp_err_t ret = i2c_master_cmd_begin(Dev.platform.port, cmd, pdMS_TO_TICKS(50)); 
        i2c_cmd_link_delete(cmd); // free command buffer memory
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr); // log device address
        }
    }

    // ----- Tof sensor setup -----
    // Don't try to reset- no pin connected
    //VL53L5CX_Reset_Sensor(&(Dev.platform));
    //ESP_LOGI(TAG, "Sensor reset done");
    
    // Check if sensor is alive 
    status = vl53l5cx_is_alive(&Dev, &isAlive);
    ESP_LOGI(TAG, "is_alive returned: status=%u isAlive=%u", status, isAlive);
    if(!isAlive || status)
    {
        ESP_LOGE(TAG, "VL53L5CX not detected at 0x%02X", Dev.platform.address);
        return;
    }

    // (Mandatory) Init VL53L5CX sensor
    status = vl53l5cx_init(&Dev);
    if(status)
    {
        ESP_LOGE(TAG, "VL53L5CX ULD Loading failed, status: %u", status);
        return;
    }

    ESP_LOGI(TAG, "VL53L5CX ULD ready! (Version: %s)", VL53L5CX_API_REVISION);

    status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
    if(status)
	{
		ESP_LOGE(TAG, "VL53L5CX failed to set resolution, status %u\n", status);
		return;
	}

    // ----- Mutex and init bool -----
    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
    tof_initialized = true;
}

uint8_t start_ranging(void)
{
    status = vl53l5cx_start_ranging(&Dev);
    if(status)
    {
        ESP_LOGE(TAG, "VL53L5CX failed to start ranging, status: %u", status);
    }
    return status;
}

uint8_t stop_ranging(void)
{
    status = vl53l5cx_stop_ranging(&Dev);
    if(status)
    {
        ESP_LOGE(TAG, "VL53L5CX failed to stop ranging, status: %u", status);
    }
    return status;
}

void tof_task(void *pvParameter)
{
    // see example in components/VL53L5CX-Library/examples/ranging_basic/main/main.c
    status = start_ranging();
    uint8_t isReady;

    while(1)
    {
        status = vl53l5cx_check_data_ready(&Dev, &isReady);

        if(isReady)
        {
            timespec_t time = getTime();

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            vl53l5cx_get_ranging_data(&Dev, &Results);
            tof_latest.results = Results;
            tof_latest.timestamp = time;
            xSemaphoreGive(dataMutex);
        }

        // Wait a few ms to avoid too high polling (function in platform file, not in API)
        VL53L5CX_WaitMs(&(Dev.platform), 20); // calls vTaskDelay under hood

        // 50 Hz = 20 ms. Changed from 5 ms
    }
}

BaseType_t tof_service(void)
{
    if (!tof_initialized)
    {
        ESP_LOGE(TAG, "ToF not initialized, cannot start service");
        return pdFAIL;
    }
    
    BaseType_t tof_status;
    tof_status = xTaskCreate(
        tof_task,
        "tof_task",
        8192,
        NULL,
        3, // prev 4
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
#include "include/rr_os_service.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "include/twai_service.h"
#include "include/led.h"
#include "freertos/queue.h" // added

// High level state machine and instructions. like EXPLORING or SEEK_NEW_PATH

// high-level configuration changes and things that were cross sensor, state estimation, requests from other modules

static const char *TAG = "EVENTS";

static QueueHandle_t event_queue;
rr_status_t status;

void initialise_events() {
    event_queue = xQueueCreate(10, sizeof(event_t));
}

void add_event(event_t event) {
    if (xQueueSend(event_queue, &event, 0) == pdPASS){
        // good
    } else {
        // 0 means if event can't be added (queue is full), return and don't add event
        ESP_LOGE(TAG, "FAILED to add event %d! Event queue full.", event);
    }
}

void rr_os_service(void *pvParameter)
{
    const char* TAG = "Events";
    event_t event; // keep rewriting this address with the events. no need to keep remaking
    while (1)
    {
        // rr_os_event_handler
        if (xQueueReceive(event_queue, &event, portMAX_DELAY)) // previously 0. pass value to return at intervals
        {
            ESP_LOGI("Events", "Event %d added to queue", event);
            switch (event)
            {
            case EVENT_CONNECTION:
                ESP_LOGI(TAG, "Connection event detected");
                // Disable the interrupt to prevent triggering on TWAI comms
                gpio_set_intr_type(TWAI_RX, GPIO_INTR_DISABLE);
                status.connected = true;
                set_led_color(CONNECTED_COLOR);
                // Uncomment line below for debugging
                //add_event(EVENT_DISCONNECT_REQUEST);
                break;
            case EVENT_DISCONNECT_REQUEST:
                ESP_LOGI(TAG, "Disconnect request event detected");
                // Enable the interrupt on falling edge to detect disconnect
                gpio_set_intr_type(TWAI_RX, GPIO_INTR_NEGEDGE);
                break;
            case EVENT_DISCONNECT:
                ESP_LOGI(TAG, "Disconnect event detected");
                set_led_color(INDEPENDENT_COLOR);
                // Change interrupt to rising edge
                status.connected = false;
                gpio_set_intr_type(TWAI_RX, GPIO_INTR_POSEDGE);
                break;
            default:
                break;
            }
        }
        //vTaskDelay(100 / portTICK_PERIOD_MS); // don't need this, queue is blocking
    }
}

void launch_rr_os_service()
{
    ESP_LOGI(TAG, "Launching rr_os_service");
    initialise_events();

    // Create the task that will handle events
    BaseType_t status;
    status = xTaskCreate(
        rr_os_service,
        "rr_os_event_handler",
        2048,
        NULL,
        4,
        NULL);

    if (status == pdPASS)
    {
        ESP_LOGI(TAG, "RR OS service started!");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting RR OS service.");
    }
}


#include "include/events.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "include/twai_service.h"
#include "include/led.h"
#include "freertos/queue.h" // added

static const char *TAG = "EVENTS";

static QueueHandle_t event_queue;
rr_state_t state;

void initialise_events() {
    event_queue = xQueueCreate(10, sizeof(event_t));
}

void add_event(event_t event) {
    xQueueSend(event_queue, &event, 0);
}
void rr_os_event_handler()
{
    const char* TAG = "Events";
    event_t event;
    if (xQueueReceive(event_queue, &event, 0))
    {
        ESP_LOGI("Events", "Event %d added to queue", event);
        switch (event)
        {
        case EVENT_CONNECTION:
            ESP_LOGI(TAG, "Connection event detected");
            // Disable the interrupt to prevent triggering on TWAI comms
            gpio_set_intr_type(TWAI_RX, GPIO_INTR_DISABLE);
            state.connected = true;
            set_led_color(CONNECTED_COLOR);
            // TODO: Remove this just for debugging
            add_event(EVENT_DISCONNECT_REQUEST);
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
            state.connected = false;
            gpio_set_intr_type(TWAI_RX, GPIO_INTR_POSEDGE);
            break;
        default:
            break;
        }
    }
}

void rr_os_service(void *pvParameter)
{
    while (1)
    {
        rr_os_event_handler();
        vTaskDelay(100 / portTICK_PERIOD_MS);
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
        ESP_LOGI(TAG, "service started");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting the service");
    }
}


#include "driver/gpio.h"
#include "include/twai_service.h"
#include "include/rr_os_service.h"
#include "freertos/task.h"

#define TAG "TWAI Service"


void twai_service_init()
{
    twai_driver_install(&gen_config, &timing_config, &filter_config);
    ESP_LOGI(TAG, "TWAI Driver Installed");
}
void twai_service_start()
{
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN Started");
}
void twai_service_stop()
{
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(TAG, "CAN Stopped");
}

void twai_service_msg_send(twai_message_t *msg)
{
    ESP_ERROR_CHECK(twai_transmit(msg, portMAX_DELAY));
    ESP_LOGI(TAG, "Message sent\tID 0x%lx\tData = %d", msg->identifier, msg->data[0]);
}
void twai_service_msg_receive(twai_message_t *msg)
{
    ESP_ERROR_CHECK(twai_receive(msg, portMAX_DELAY));
    ESP_LOGI(TAG, "Message received\tID 0x%lx\tData = %d", msg->identifier, msg->data[0]);
}

void create_msg(twai_message_t *msg, uint32_t id, uint8_t data)
{
    msg->extd = 0;
    msg->rtr = 0;
    msg->ss = 0;
    msg->self = 1;
    msg->dlc_non_comp = 0;
    msg->identifier = id;
    msg->data_length_code = 1;
    msg->data[0] = data;
}

void twai_listener(void)
{
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK)
    {
        ESP_LOGI(TAG, "Message received");
        ESP_LOGI(TAG, "ID: 0x%lx", message.identifier);
    }
    else
    {
        printf("Failed to receive message\n");
        return;
    }
}


static void IRAM_ATTR twai_service_isr_handler(void *arg)
{
    if (xTaskGetTickCount() - rr_status.last_isr_time < pdMS_TO_TICKS(100))
    {
        return;
    }
    rr_status.last_isr_time = xTaskGetTickCount();
    if (rr_status.connected)
    {
        add_event(EVENT_DISCONNECT);
    }
    else
    {
        add_event(EVENT_CONNECTION);
    }
}

void twai_interrupt_init()
{
    // Set the TWAI RX pin as an input pin
    // gpio_pad_select_gpio(TWAI_RX);
    gpio_set_direction(TWAI_RX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TWAI_RX, GPIO_PULLDOWN_ONLY);
    // Set the TWAI RX pin as an interrupt pin
    gpio_set_intr_type(TWAI_RX, GPIO_INTR_POSEDGE);
    // Install the driver's GPIO ISR handler service, which allows per-pin GPIO interrupt handlers
    // gpio_install_isr_service(0); // Already installed for imu
    // Hook up the TWAI RX pin to the ISR handler
    gpio_isr_handler_add(TWAI_RX, twai_service_isr_handler, NULL);
}

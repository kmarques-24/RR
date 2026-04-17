/*
TWAI Service for monitoring CAN bus traffic
*/

#pragma once

#include "driver/twai.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TWAI_TX (gpio_num_t) CONFIG_TWAI_TX
#define TWAI_RX (gpio_num_t) CONFIG_TWAI_RX

// static bool twai_active = false;

// Configuring TWAI
const twai_general_config_t gen_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX, TWAI_RX, TWAI_MODE_NO_ACK); // Only No_ACK for self test
// Timing configuration
const twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_25KBITS();
// Filtering configuration
const twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void twai_service_init();
void twai_service_start();
void twai_service_stop();

void twai_service_msg_send(twai_message_t *msg);
void twai_service_msg_receive(twai_message_t *msg);

void create_msg(twai_message_t *msg, uint32_t id, uint8_t data);

void twai_interrupt_init();

#ifdef __cplusplus
}
#endif
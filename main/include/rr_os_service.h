#pragma once

#include <stdbool.h>
#include <inttypes.h>

typedef enum EVENTS {
    EVENT_NONE = 0,
    EVENT_CONNECTION,
    EVENT_DISCONNECT_REQUEST,
    EVENT_DISCONNECT,
} event_t;

typedef struct rr_state_t {
    bool twai_active;
    bool connected;
    uint32_t last_isr_time;
    bool leader;
    bool imu_enabled;
    bool led_enabled;
    bool radio_enabled;
    bool encoder_enabled;
    bool wifi_enabled;
} rr_state_t;

extern rr_state_t state;

void initialise_events();
void add_event(event_t event);
void rr_os_event_handler();
void rr_os_service(void *pvParameter);
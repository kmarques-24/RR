#pragma once

#include <stdbool.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum EVENTS {
    EVENT_NONE = 0,
    EVENT_CONNECTION,
    EVENT_DISCONNECT_REQUEST,
    EVENT_DISCONNECT,
} event_t;

typedef struct rr_status_t {
    bool twai_active;
    bool connected;
    uint32_t last_isr_time;
    bool leader;
    bool imu_enabled;
    bool led_enabled;
    bool radio_enabled;
    bool encoder_enabled;
    bool wifi_enabled;
} rr_status_t;

extern rr_status_t status;

void initialise_events();
void add_event(event_t event);
void rr_os_event_handler();
void launch_rr_os_service(void);

#ifdef __cplusplus
}
#endif
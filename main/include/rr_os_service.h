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
    bool encoder_enabled;
    bool drive_enabled;
    bool estimator_enabled;
    bool wifi_enabled;
    bool imu_enabled;
    bool tof_enabled;
    bool key_control_enabled;
    bool uros_enabled;

    bool twai_active;
    bool connected;
    uint32_t last_isr_time;
    bool leader;
    bool led_enabled;
    bool radio_enabled;
} rr_status_t;

extern rr_status_t rr_status;

void initialise_events();
void add_event(event_t event);
void rr_os_event_handler();
void launch_rr_os_service(void);

#ifdef __cplusplus
}
#endif
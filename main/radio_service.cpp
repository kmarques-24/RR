#include "radio_service.h"

// TODO: Add params to Kconfig menu
// TODO: Get data to triangulate position
// TODO: Update estimator position estimate with radio input. May need EKF

// ISR handler must not use non-ISR-safe functions like `gpio_get_level` unless GPIO is input-only and stable
/*
void initialise_radio()
{
    CLK, MISO, MOSI, CS
    RadioLibCustomHAL hal = RadioLibCustomHAL(1, 2, 3, 5);
    // CS, G0, RST
    SX1278 radio = new Module(&hal, 5, 7, 4);
    int rr_status = radio.begin();
    if (rr_status == RADIOLIB_ERR_NONE)
    {
        ESP_LOGI("Radio", "Radio initialised successfully");
    }
    else {ESP_LOGE("Radio", "Radio failed to initialise");}

    radio.setFrequency(433.0);
    radio.setSpreadingFactor(12);
}
*/
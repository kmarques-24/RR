#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ESP32S3_ZERO

// Digital Pins
#define D0 GPIO_NUM_0   // Boot button (use carefully)
#define D1 GPIO_NUM_1   // TXD0 (Serial)
#define D2 GPIO_NUM_2   // RXD0 (Serial)
#define D3 GPIO_NUM_3   // SPI CS
#define D4 GPIO_NUM_4   // SPI SCK
#define D5 GPIO_NUM_5   // SPI MOSI
#define D6 GPIO_NUM_6   // SPI MISO
#define D7 GPIO_NUM_7   // Not broken out
#define D8 GPIO_NUM_8   // Not broken out
#define D9 GPIO_NUM_9   // Not broken out
#define D10 GPIO_NUM_10 // Not broken out

// Analog Pins (ADC1 channels)
#define A0 GPIO_NUM_1   // ADC1_CH0 (shared with D1/TXD0)
#define A1 GPIO_NUM_2   // ADC1_CH1 (shared with D2/RXD0)
#define A2 GPIO_NUM_3   // ADC1_CH2 (shared with D3)
#define A3 GPIO_NUM_4   // ADC1_CH3 (shared with D4)

// Special Function Pins
#define LED_BUILTIN GPIO_NUM_13  // Onboard LED
#define BOOT_BUTTON GPIO_NUM_0   // Boot button

// Warning: Pins used for onboard functions
#define USB_DM GPIO_NUM_19  // USB D- (don't use)
#define USB_DP GPIO_NUM_20  // USB D+ (don't use)

#endif // ESP32S3_ZERO

#ifdef __cplusplus
}
#endif
#include "include/led.h"
#include "esp_log.h"

static led_indicator_handle_t led_handle;

const led_strip_config_t strip_config = {
    .strip_gpio_num = LED_PIN,
    .max_leds = 1,
    .led_pixel_format = LED_PIXEL_FORMAT_GRB,
    .led_model = LED_MODEL_WS2812,
    .flags = {}
};

// RMT is the remote control transceiver peripheral
// We are abusing it, as is standard, to drive the LED strip
const led_strip_rmt_config_t rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = LED_FREQ,
    .mem_block_symbols = {},
    .flags = {}
};

led_indicator_strips_config_t strips_config = {
    .led_strip_cfg = strip_config,
    .led_strip_driver = LED_STRIP_RMT,
    .led_strip_rmt_cfg = rmt_config,
};

const led_indicator_config_t led_config = {
    .mode = LED_STRIPS_MODE,
    .led_indicator_strips_config = &strips_config,
    .blink_lists = NULL, // We are not creating a custom list of blinking patterns
    .blink_list_num = 0,
};


void initialise_led()
{
    led_handle = led_indicator_create(&led_config);
    led_indicator_set_brightness(led_handle, BRIGHTNESS);
}

void set_led_color(int32_t color)
{
    led_indicator_set_rgb(led_handle, color);
    ESP_LOGI("LED", "Color set to %ld", color);
}
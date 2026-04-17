/*
#pragma once

#include <RadioLib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define L_TAG "Esp32S3Hal"

#define RADIO_HOST SPI2_HOST

#define NOP() asm volatile("nop")

#define INPUT GPIO_MODE_INPUT
#define OUTPUT GPIO_MODE_OUTPUT
#define LOW 0
#define HIGH 1
#define RISING GPIO_INTR_POSEDGE
#define FALLING GPIO_INTR_NEGEDGE


class RadioLibCustomHAL : public RadioLibHal
{
public:
    RadioLibCustomHAL(int8_t sck, int8_t miso, int8_t mosi, int8_t cs)
        : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
          spiSCK(sck), spiMISO(miso), spiMOSI(mosi), spiCS(cs)
    {
    }

    void init() override
    {
        spiBegin();
    }

    void term() override
    {
        spiEnd();
    }

    void pinMode(uint32_t pin, uint32_t mode) override
    {
        if (pin == RADIOLIB_NC)
        {
            return;
        }

        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = (gpio_mode_t)mode,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        gpio_config(&conf);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override
    {
        if (pin == RADIOLIB_NC)
        {
            return;
        }
        gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override
    {
        if (pin == RADIOLIB_NC)
        {
            return (0);
        }

        return (gpio_get_level((gpio_num_t)pin));
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override
    {
        if (interruptNum == RADIOLIB_NC)
        {
            return;
        }

        gpio_install_isr_service((int)ESP_INTR_FLAG_IRAM);
        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));

        // this uses function typecasting, which is not defined when the functions have different signatures
        // untested and might not work
        gpio_isr_handler_add((gpio_num_t)interruptNum, (void (*)(void *))interruptCb, NULL);
    }

    void detachInterrupt(uint32_t interruptNum) override
    {
        if (interruptNum == RADIOLIB_NC)
        {
            return;
        }

        gpio_isr_handler_remove((gpio_num_t)interruptNum);
        gpio_wakeup_disable((gpio_num_t)interruptNum);
        gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
    }

    void delay(unsigned long ms) override
    {
        vTaskDelay(ms / portTICK_PERIOD_MS);
    }

    void delayMicroseconds(unsigned long us) override
    {
        uint64_t m = (uint64_t)esp_timer_get_time();
        if (us)
        {
            uint64_t e = (m + us);
            if (m > e)
            { // overflow
                while ((uint64_t)esp_timer_get_time() > e)
                {
                    NOP();
                }
            }
            while ((uint64_t)esp_timer_get_time() < e)
            {
                NOP();
            }
        }
    }

    unsigned long millis() override
    {
        return ((unsigned long)(esp_timer_get_time() / 1000ULL));
    }

    unsigned long micros() override
    {
        return ((unsigned long)(esp_timer_get_time()));
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override
    {
        if (pin == RADIOLIB_NC)
        {
            return (0);
        }

        this->pinMode(pin, INPUT);
        uint32_t start = this->micros();
        uint32_t curtick = this->micros();

        while (this->digitalRead(pin) == state)
        {
            if ((this->micros() - curtick) > timeout)
            {
                return (0);
            }
        }

        return (this->micros() - start);
    }

    void spiBegin()
    {
        spi_bus_config_t bus_config = {
            .mosi_io_num = this->spiMOSI,
            .miso_io_num = this->spiMISO,
            .sclk_io_num = this->spiSCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4096 // Arbitrary Magic number
        };

        spi_device_interface_config_t dev_config = {
            .mode = 0,
            .clock_speed_hz = 1000000,
            .spics_io_num = this->spiCS,
            .queue_size = 10,
            .pre_cb = NULL,
            .post_cb = NULL

        };
        ESP_ERROR_CHECK(spi_bus_initialize(RADIO_HOST, &bus_config, SPI_DMA_CH_AUTO));
        ESP_ERROR_CHECK(spi_bus_add_device(RADIO_HOST, &dev_config, &this->spi));
    }

    void spiBeginTransaction() {}
    void spiTransferByte(uint8_t b) { (void)b; }
    void spiEndTransaction() {}

    void spiEnd()
    {
        // spi_bus_free(this->spi);
    }

    void spiTransfer(uint8_t *out, size_t len, uint8_t *in)
    {
        spi_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.length = len * 8; // Length in bits not bytes
        trans.tx_buffer = out;
        trans.rx_buffer = in;
        ESP_ERROR_CHECK(spi_device_transmit(this->spi, &trans));
    }

private:
    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;
    int8_t spiCS;
    spi_device_handle_t spi;
};

#ifdef __cplusplus
}
#endif

*/
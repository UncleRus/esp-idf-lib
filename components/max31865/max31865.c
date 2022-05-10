/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file max31865.c
 *
 * ESP-IDF driver for MAX31865, resistance converter for platinum RTDs
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "max31865.h"
#include <string.h>
#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "max31865";

#define REG_CONFIG         (0x00)
#define REG_RTD_MSB        (0x01)
#define REG_HIGH_FAULT_MSB (0x03)
#define REG_LOW_FAULT_MSB  (0x05)
#define REG_FAULT_STATUS   (0x07)

#define BIT_CONFIG_50HZ        BIT(0)
#define BIT_CONFIG_FAULT_CLEAR BIT(1)
#define BIT_CONFIG_FAULT_D2    BIT(2)
#define BIT_CONFIG_FAULT_D3    BIT(3)
#define BIT_CONFIG_3WIRE       BIT(4)
#define BIT_CONFIG_1SHOT       BIT(5)
#define BIT_CONFIG_AUTO        BIT(6)
#define BIT_CONFIG_VBIAS       BIT(7)

#define MASK_CONFIG_FAULT (BIT_CONFIG_FAULT_D2 | BIT_CONFIG_FAULT_D3)

typedef struct
{
    float a, b;
} rtd_coeff_t;

static const rtd_coeff_t rtd_coeff[] = {
     [MAX31865_ITS90]         = { .a = 3.9083e-3f, .b = -5.775e-7f },
     [MAX31865_DIN43760]      = { .a = 3.9848e-3f, .b = -5.8019e-7f },
     [MAX31865_US_INDUSTRIAL] = { .a = 3.9692e-3f, .b = -5.8495e-7f },
};

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t write_reg_8(max31865_t *dev, uint8_t reg, uint8_t val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t tx[] = { reg | 0x80, val };

    t.tx_buffer = tx;
    t.length = sizeof(tx) * 8;

    return spi_device_transmit(dev->spi_dev, &t);
}

static esp_err_t read_reg_8(max31865_t *dev, uint8_t reg, uint8_t *val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t tx[] = { reg, 0 };
    uint8_t rx[sizeof(tx)];

    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.length = sizeof(tx) * 8;
    CHECK(spi_device_transmit(dev->spi_dev, &t));

    *val = rx[1];

    return ESP_OK;
}

static esp_err_t read_reg_16(max31865_t *dev, uint8_t reg, uint16_t *val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t tx[] = { reg, 0, 0 };
    uint8_t rx[sizeof(tx)];

    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.length = sizeof(tx) * 8;
    CHECK(spi_device_transmit(dev->spi_dev, &t));

    *val = (uint16_t)rx[1] << 8;
    *val |= rx[2];

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t max31865_init_desc(max31865_t *dev, spi_host_device_t host, uint32_t clock_speed_hz, gpio_num_t cs_pin)
{
    CHECK_ARG(dev);

    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = cs_pin;
    dev->spi_cfg.clock_speed_hz = clock_speed_hz;
    dev->spi_cfg.mode = 1;
    dev->spi_cfg.queue_size = 1;
    dev->spi_cfg.cs_ena_pretrans = 1;
    //dev->spi_cfg.flags = SPI_DEVICE_NO_DUMMY;

    return spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
}

esp_err_t max31865_free_desc(max31865_t *dev)
{
    CHECK_ARG(dev);

    return spi_bus_remove_device(dev->spi_dev);
}

esp_err_t max31865_set_config(max31865_t *dev, const max31865_config_t *config)
{
    CHECK_ARG(dev && config);

    uint8_t val;
    CHECK(read_reg_8(dev, REG_CONFIG, &val));

    val &= ~(BIT_CONFIG_AUTO | BIT_CONFIG_3WIRE | BIT_CONFIG_VBIAS | BIT_CONFIG_50HZ);

    val |= config->mode == MAX31865_MODE_AUTO ? BIT_CONFIG_AUTO : 0;
    val |= config->connection == MAX31865_3WIRE ? BIT_CONFIG_3WIRE : 0;
    val |= config->v_bias ? BIT_CONFIG_VBIAS : 0;
    val |= config->filter == MAX31865_FILTER_50HZ ? BIT_CONFIG_50HZ : 0;

    CHECK(write_reg_8(dev, REG_CONFIG, val));

    return ESP_OK;
}

esp_err_t max31865_get_config(max31865_t *dev, max31865_config_t *config)
{
    CHECK_ARG(dev && config);

    uint8_t val;
    CHECK(read_reg_8(dev, REG_CONFIG, &val));

    config->filter = val & BIT_CONFIG_50HZ ? MAX31865_FILTER_50HZ : MAX31865_FILTER_60HZ;
    config->v_bias = val & BIT_CONFIG_VBIAS ? 1 : 0;
    config->mode = val & BIT_CONFIG_AUTO ? MAX31865_MODE_AUTO : MAX31865_MODE_SINGLE;
    config->connection = val & BIT_CONFIG_3WIRE ? MAX31865_3WIRE : MAX31865_2WIRE;

    return ESP_OK;
}

esp_err_t max31865_start_measurement(max31865_t *dev)
{
    CHECK_ARG(dev);

    uint8_t val;
    CHECK(read_reg_8(dev, REG_CONFIG, &val));
    val |= BIT_CONFIG_1SHOT;
    CHECK(write_reg_8(dev, REG_CONFIG, val));

    return ESP_OK;
}

esp_err_t max31865_read_raw(max31865_t *dev, uint16_t *raw, bool *fault)
{
    CHECK_ARG(dev && raw && fault);

    CHECK(read_reg_16(dev, REG_RTD_MSB, raw));
    *fault = *raw & 1;
    *raw >>= 1;

    return ESP_OK;
}

// conversion code taken from Adafruit library
esp_err_t max31865_read_temperature(max31865_t *dev, float *temp)
{
    CHECK_ARG(dev && temp && dev->standard <= MAX31865_US_INDUSTRIAL);

    uint16_t raw;
    bool fault;
    CHECK(max31865_read_raw(dev, &raw, &fault));
    if (fault)
    {
        ESP_LOGE(TAG, "[CS %d] Fault detected", dev->spi_cfg.spics_io_num);
        return ESP_FAIL;
    }

    float r_rtd = raw * dev->r_ref / 32768;

    ESP_LOGD(TAG, "[CS %d] RTD resistance: %.8f", dev->spi_cfg.spics_io_num, r_rtd);

    const rtd_coeff_t *c = rtd_coeff + dev->standard;

    *temp = (sqrtf((c->a * c->a - (4 * c->b)) + (4 * c->b / dev->rtd_nominal * r_rtd)) - c->a) / (2 * c->b);

    if (*temp >= 0)
        return ESP_OK;

    // below zero
    r_rtd = r_rtd / dev->rtd_nominal * 100; // normalize to 100 Ohm

    float rpoly = r_rtd;

    *temp = -242.02;
    *temp += 2.2228 * rpoly;
    rpoly *= r_rtd; // square
    *temp += 2.5859e-3 * rpoly;
    rpoly *= r_rtd; // ^3
    *temp -= 4.8260e-6 * rpoly;
    rpoly *= r_rtd; // ^4
    *temp -= 2.8183e-8 * rpoly;
    rpoly *= r_rtd; // ^5
    *temp += 1.5243e-10 * rpoly;

    return ESP_OK;
}

esp_err_t max31865_measure(max31865_t *dev, float *temp)
{
    CHECK(max31865_start_measurement(dev));
    vTaskDelay(pdMS_TO_TICKS(70));
    return max31865_read_temperature(dev, temp);
}

esp_err_t max31865_detect_fault_auto(max31865_t *dev)
{
    CHECK_ARG(dev);

    uint8_t conf;
    CHECK(read_reg_8(dev, REG_CONFIG, &conf));

    uint8_t fault_bits = conf & MASK_CONFIG_FAULT;
    if (fault_bits == BIT_CONFIG_FAULT_D2)
    {
        ESP_LOGD(TAG, "[CS %d] Automatic fault detection still running", dev->spi_cfg.spics_io_num);
        return ESP_ERR_INVALID_STATE;
    }
    if (fault_bits == BIT_CONFIG_FAULT_D3)
    {
        ESP_LOGD(TAG, "[CS %d] Manual cycle 1 still running, waiting for user to start cycle 2", dev->spi_cfg.spics_io_num);
        return ESP_ERR_INVALID_STATE;
    }
    if (fault_bits == (BIT_CONFIG_FAULT_D2 | BIT_CONFIG_FAULT_D3))
    {
        ESP_LOGD(TAG, "[CS %d] Manual cycle 2 still running", dev->spi_cfg.spics_io_num);
        return ESP_ERR_INVALID_STATE;
    }

    CHECK(write_reg_8(dev, REG_CONFIG, BIT_CONFIG_VBIAS | BIT_CONFIG_FAULT_D2));

    uint8_t tmp;
    do
    {
        // FIXME endless loop
        CHECK(read_reg_8(dev, REG_CONFIG, &tmp));
    }
    while (tmp & MASK_CONFIG_FAULT);

    return ESP_OK;
}

esp_err_t max31865_get_fault_status(max31865_t *dev, uint8_t *fault_status)
{
    CHECK_ARG(dev && fault_status);

    return read_reg_8(dev, REG_FAULT_STATUS, fault_status);
}

esp_err_t max31865_clear_fault_status(max31865_t *dev)
{
    CHECK_ARG(dev);

    uint8_t val;
    CHECK(read_reg_8(dev, REG_CONFIG, &val));
    val &= ~(BIT_CONFIG_1SHOT | BIT_CONFIG_FAULT_D2 | BIT_CONFIG_FAULT_D3);
    val |= BIT_CONFIG_FAULT_CLEAR;
    CHECK(write_reg_8(dev, REG_CONFIG, val));

    return ESP_OK;
}

/*
 * Copyright (c) 2017 Brian Schwind <https://github.com/bschwind>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file tsl4531.c
 *
 * ESP-IDF driver for digital ambient light sensor TSL4531
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2017 Brian Schwind <https://github.com/bschwind>\n
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include "tsl4531.h"

#define I2C_FREQ_HZ 400000

static const char *TAG = "tsl4531";

// Registers
#define TSL4531_REG_COMMAND   0x80
#define TSL4531_REG_CONTROL   0x00
#define TSL4531_REG_CONFIG    0x01
#define TSL4531_REG_DATA_LOW  0x04
#define TSL4531_REG_DATA_HIGH 0x05
#define TSL4531_REG_DEVICE_ID 0x0A

// TSL4531 Misc Values
#define TSL4531_ON  0x03
#define TSL4531_OFF 0x00

// Integration times in milliseconds
#define TSL4531_INTEGRATION_TIME_100MS 120
#define TSL4531_INTEGRATION_TIME_200MS 240
#define TSL4531_INTEGRATION_TIME_400MS 480 // Default

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

inline static esp_err_t write_register(tsl4531_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(&dev->i2c_dev, TSL4531_REG_COMMAND | reg, &val, 1);
}

inline static esp_err_t read_register(tsl4531_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(&dev->i2c_dev, TSL4531_REG_COMMAND | reg, val, 1);
}

inline static esp_err_t read_register_16(tsl4531_t *dev, uint8_t reg, uint16_t *val)
{
    return i2c_dev_read_reg(&dev->i2c_dev, TSL4531_REG_COMMAND | reg, val, 2);
}

inline static esp_err_t enable(tsl4531_t *dev)
{
    return write_register(dev, TSL4531_REG_CONTROL, TSL4531_ON);
}

inline static esp_err_t disable(tsl4531_t *dev)
{
    return write_register(dev, TSL4531_REG_CONTROL, TSL4531_OFF);
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t tsl4531_init_desc(tsl4531_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = TSL4531_I2C_ADDR;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t tsl4531_free_desc(tsl4531_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t tsl4531_init(tsl4531_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, enable(dev));

    uint8_t control_reg;
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL4531_REG_CONTROL, &control_reg));
    if (control_reg != TSL4531_ON)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        ESP_LOGE(TAG, "Error initializing TSL4531, control register wasn't set to ON");
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t id;
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL4531_REG_DEVICE_ID, &id));
    id >>= 4;
    switch (id)
    {
        case TSL4531_PART_TSL45317:
        case TSL4531_PART_TSL45313:
        case TSL4531_PART_TSL45315:
        case TSL4531_PART_TSL45311:
            dev->part_id = id;
            break;
        default:
            ESP_LOGW(TAG, "Unknown part id for TSL4531 sensor: %u", id);
    }

    I2C_DEV_CHECK(&dev->i2c_dev, disable(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl4531_config(tsl4531_t *dev, tsl4531_integration_time_t integration_time, bool skip_power_save)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, enable(dev));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL4531_REG_CONFIG,
            (skip_power_save ? 0x08 : 0x00) | (0x03 & integration_time)));
    I2C_DEV_CHECK(&dev->i2c_dev, disable(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->integration_time = integration_time;
    dev->skip_power_save = skip_power_save;

    return ESP_OK;
}

esp_err_t tsl4531_read_lux(tsl4531_t *dev, uint16_t *lux)
{
    CHECK_ARG(dev && lux);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, enable(dev));

    uint16_t multiplier;
    switch (dev->integration_time)
    {
        case TSL4531_INTEGRATION_100MS:
            multiplier = 4;
            vTaskDelay(pdMS_TO_TICKS(TSL4531_INTEGRATION_TIME_100MS));
            break;
        case TSL4531_INTEGRATION_200MS:
            multiplier = 2;
            vTaskDelay(pdMS_TO_TICKS(TSL4531_INTEGRATION_TIME_200MS));
            break;
        default:
            multiplier = 1;
            vTaskDelay(pdMS_TO_TICKS(TSL4531_INTEGRATION_TIME_400MS));
    }

    uint16_t lux_data;
    I2C_DEV_CHECK(&dev->i2c_dev, read_register_16(dev, TSL4531_REG_DATA_LOW, &lux_data));
    I2C_DEV_CHECK(&dev->i2c_dev, disable(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *lux = multiplier * lux_data;

    return ESP_OK;
}

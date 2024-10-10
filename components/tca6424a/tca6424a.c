/*
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file tca6424a.c
 *
 * ESP-IDF driver for TCA6424A low-voltage 24-bit I2C I/O expander
 *
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include "tca6424a.h"
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>

#define I2C_FREQ_HZ 400000

#define REG_AI_PREFIX BIT(7)

#define REG_INPUT_0  0x00
#define REG_INPUT_1  0x01
#define REG_INPUT_2  0x02
#define REG_OUTPUT_0 0x04
#define REG_OUTPUT_1 0x05
#define REG_OUTPUT_2 0x06
#define REG_POL_0    0x08
#define REG_POL_1    0x09
#define REG_POL_2    0x0a
#define REG_CONF_0   0x0c
#define REG_CONF_1   0x0d
#define REG_CONF_2   0x0e

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "tca6424a";

static esp_err_t read_reg_24(i2c_dev_t *dev, uint8_t reg, uint32_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, val, 3));
    I2C_DEV_GIVE_MUTEX(dev);

    *val &= 0x00ffffff;

    return ESP_OK;
}

static esp_err_t write_reg_24(i2c_dev_t *dev, uint8_t reg, uint32_t val)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &val, 3));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t tca6424a_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != TCA6424A_I2C_ADDRESS_GND && addr != TCA6424A_I2C_ADDRESS_VCC)
    {
        ESP_LOGE(TAG, "Invalid I2C address: 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t tca6424a_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t tca6424a_port_get_mode(i2c_dev_t *dev, uint32_t *mode)
{
    return read_reg_24(dev, REG_CONF_0 | REG_AI_PREFIX, mode);
}

esp_err_t tca6424a_port_set_mode(i2c_dev_t *dev, uint32_t mode)
{
    return write_reg_24(dev, REG_CONF_0 | REG_AI_PREFIX, mode);
}

esp_err_t tca6424a_port_get_polarity_inversion(i2c_dev_t *dev, uint32_t *polarity)
{
    return read_reg_24(dev, REG_POL_0 | REG_AI_PREFIX, polarity);
}

esp_err_t tca6424a_port_set_polarity_inversion(i2c_dev_t *dev, uint32_t polarity)
{
    return write_reg_24(dev, REG_POL_0 | REG_AI_PREFIX, polarity);
}

esp_err_t tca6424a_port_read(i2c_dev_t *dev, uint32_t *val)
{
    return read_reg_24(dev, REG_INPUT_0 | REG_AI_PREFIX, val);
}

esp_err_t tca6424a_port_write(i2c_dev_t *dev, uint32_t val)
{
    return write_reg_24(dev, REG_OUTPUT_0 | REG_AI_PREFIX, val);
}

esp_err_t tca6424a_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t *val)
{
    CHECK_ARG(dev && val && pin < 24);

    uint8_t raw;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_INPUT_0 + pin / 8, &raw, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *val = raw & BIT(pin % 8) ? 1 : 0;

    return ESP_OK;
}

esp_err_t tca6424a_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val)
{
    CHECK_ARG(dev && pin < 24);

    uint8_t raw;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_OUTPUT_0 + pin / 8, &raw, 1));
    raw &= ~BIT(pin % 8);
    if (val)
        raw |= BIT(pin % 8);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_OUTPUT_0 + pin / 8, &raw, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

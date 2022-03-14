/*
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
 * @file tca95x5.c
 *
 * ESP-IDF driver for TCA9535/TCA9555 remote 16-bit I/O expanders for I2C-bus
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_idf_lib_helpers.h>
#include "tca95x5.h"

#define I2C_FREQ_HZ 400000

#define REG_IN0   0x00
#define REG_OUT0  0x02
#define REG_CONF0 0x06

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

static esp_err_t read_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, val, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t write_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t val)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &val, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t tca95x5_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev && (addr & TCA95X5_I2C_ADDR_BASE));

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t tca95x5_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t tca95x5_port_get_mode(i2c_dev_t *dev, uint16_t *mode)
{
    return read_reg_16(dev, REG_CONF0, mode);
}

esp_err_t tca95x5_port_set_mode(i2c_dev_t *dev, uint16_t mode)
{
    return write_reg_16(dev, REG_CONF0, mode);
}

esp_err_t tca95x5_port_read(i2c_dev_t *dev, uint16_t *val)
{
    return read_reg_16(dev, REG_IN0, val);
}

esp_err_t tca95x5_port_write(i2c_dev_t *dev, uint16_t val)
{
    return write_reg_16(dev, REG_OUT0, val);
}

esp_err_t tca95x5_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t *val)
{
    uint16_t v;
    CHECK(read_reg_16(dev, REG_IN0, &v));
    *val = v & BV(pin) ? 1 : 0;

    return ESP_OK;
}

esp_err_t tca95x5_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val)
{
    uint16_t v;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_OUT0, &v, 2));
    v = (v & ~BV(pin)) | (val ? BV(pin) : 0);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_OUT0, &v, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}


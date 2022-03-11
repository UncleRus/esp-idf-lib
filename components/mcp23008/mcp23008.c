/*
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file mcp23008.c
 *
 * ESP-IDF driver for I2C 8 bit GPIO expander MCP23008
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "mcp23008.h"

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 1.7Mhz

#define REG_IODIR   0x00
#define REG_IPOL    0x01
#define REG_GPINTEN 0x02
#define REG_DEFVAL  0x03
#define REG_INTCON  0x04
#define REG_IOCON   0x05
#define REG_GPPU    0x06
#define REG_INTF    0x07
#define REG_INTCAP  0x08
#define REG_GPIO    0x09
#define REG_OLAT    0x0a

#define BIT_IOCON_INTPOL 1
#define BIT_IOCON_ODR    2
//#define BIT_IOCON_HAEN   3
#define BIT_IOCON_DISSLW 4
#define BIT_IOCON_SREAD  5

static const char *TAG = "mcp23008";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

static esp_err_t read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t read_reg_bit(i2c_dev_t *dev, uint8_t reg, bool *val, uint8_t bit)
{
    CHECK_ARG(dev && val);

    uint8_t buf;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, &buf, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *val = (buf & BV(bit)) >> bit;

    return ESP_OK;
}

static esp_err_t write_reg_bit(i2c_dev_t *dev, uint8_t reg, bool val, uint8_t bit)
{
    CHECK_ARG(dev);

    uint8_t buf;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, &buf, 1));
    buf = (buf & ~BV(bit)) | (val ? BV(bit) : 0);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &buf, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t mcp23008_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr < MCP23008_I2C_ADDR_BASE || addr > MCP23008_I2C_ADDR_BASE + 7)
    {
        ESP_LOGE(TAG, "Invalid device address: 0x%02x", addr);
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

esp_err_t mcp23008_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t mcp23008_get_int_out_mode(i2c_dev_t *dev, mcp23008_int_out_mode_t *mode)
{
    CHECK_ARG(mode);

    uint8_t r;
    CHECK(read_reg(dev, REG_IOCON, &r));

    if (r & BV(BIT_IOCON_ODR))
    {
        *mode = MCP23008_OPEN_DRAIN;
        return ESP_OK;
    }

    *mode = r & BV(BIT_IOCON_INTPOL) ? MCP23008_ACTIVE_HIGH : MCP23008_ACTIVE_LOW;
    return ESP_OK;
}

esp_err_t mcp23008_set_int_out_mode(i2c_dev_t *dev, mcp23008_int_out_mode_t mode)
{
    if (mode == MCP23008_OPEN_DRAIN)
        return write_reg_bit(dev, REG_IOCON, true, BIT_IOCON_ODR);

    return write_reg_bit(dev, REG_IOCON, mode == MCP23008_ACTIVE_HIGH, BIT_IOCON_INTPOL);
}

esp_err_t mcp23008_port_get_mode(i2c_dev_t *dev, uint8_t *val)
{
    return read_reg(dev, REG_IODIR, val);
}

esp_err_t mcp23008_port_set_mode(i2c_dev_t *dev, uint8_t val)
{
    return write_reg(dev, REG_IODIR, val);
}

esp_err_t mcp23008_port_get_pullup(i2c_dev_t *dev, uint8_t *val)
{
    return read_reg(dev, REG_GPPU, val);
}

esp_err_t mcp23008_port_set_pullup(i2c_dev_t *dev, uint8_t val)
{
    return write_reg(dev, REG_GPPU, val);
}

esp_err_t mcp23008_port_read(i2c_dev_t *dev, uint8_t *val)
{
    return read_reg(dev, REG_GPIO, val);
}

esp_err_t mcp23008_port_write(i2c_dev_t *dev, uint8_t val)
{
    return write_reg(dev, REG_GPIO, val);
}

esp_err_t mcp23008_get_mode(i2c_dev_t *dev, uint8_t pin, mcp23008_gpio_mode_t *mode)
{
    CHECK_ARG(mode && pin < 8);

    bool buf;
    CHECK(read_reg_bit(dev, REG_IODIR, &buf, pin));
    *mode = buf ? MCP23008_GPIO_INPUT : MCP23008_GPIO_OUTPUT;

    return ESP_OK;
}

esp_err_t mcp23008_set_mode(i2c_dev_t *dev, uint8_t pin, mcp23008_gpio_mode_t mode)
{
    CHECK_ARG(pin < 8);

    return write_reg_bit(dev, REG_IODIR, mode, pin);
}

esp_err_t mcp23008_get_pullup(i2c_dev_t *dev, uint8_t pin, bool *enable)
{
    CHECK_ARG(pin < 8);

    return read_reg_bit(dev, REG_GPPU, enable, pin);
}

esp_err_t mcp23008_set_pullup(i2c_dev_t *dev, uint8_t pin, bool enable)
{
    CHECK_ARG(pin < 8);

    return write_reg_bit(dev, REG_GPPU, enable, pin);
}

esp_err_t mcp23008_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t *val)
{
    CHECK_ARG(val && pin < 8);

    bool buf;
    CHECK(read_reg_bit(dev, REG_GPIO, &buf, pin));
    *val = buf ? 1 : 0;

    return ESP_OK;
}

esp_err_t mcp23008_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val)
{
    CHECK_ARG(pin < 8);

    return write_reg_bit(dev, REG_GPIO, val, pin);
}

esp_err_t mcp23008_port_set_interrupt(i2c_dev_t *dev, uint8_t mask, mcp23008_gpio_intr_t intr)
{
    CHECK_ARG(dev);

    uint8_t int_en;
    CHECK(read_reg(dev, REG_GPINTEN, &int_en));

    if (intr == MCP23008_INT_DISABLED)
    {
        // disable interrupts
        int_en &= ~mask;
        CHECK(write_reg(dev, REG_GPINTEN, int_en));

        return ESP_OK;
    }

    uint8_t int_con;
    CHECK(read_reg(dev, REG_INTCON, &int_con));

    if (intr == MCP23008_INT_ANY_EDGE)
        int_con &= ~mask;
    else
    {
        int_con |= mask;

        uint8_t int_def;
        CHECK(read_reg(dev, REG_DEFVAL, &int_def));
        if (intr == MCP23008_INT_LOW_EDGE)
            int_def |= mask;
        else
            int_def &= ~mask;
        CHECK(write_reg(dev, REG_DEFVAL, int_def));
    }

    CHECK(write_reg(dev, REG_INTCON, int_con));

    // enable interrupts
    int_en |= mask;
    CHECK(write_reg(dev, REG_GPINTEN, int_en));

    return ESP_OK;
}

esp_err_t mcp23008_set_interrupt(i2c_dev_t *dev, uint8_t pin, mcp23008_gpio_intr_t intr)
{
    CHECK_ARG(pin < 8);

    return mcp23008_port_set_interrupt(dev, BV(pin), intr);
}

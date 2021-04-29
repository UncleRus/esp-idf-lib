/*
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file ds1307.c
 *
 * ESP-IDF driver for DS1307 real-time clock
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include "ds1307.h"

#define I2C_FREQ_HZ 400000

#define RAM_SIZE 56

#define TIME_REG    0
#define CONTROL_REG 7
#define RAM_REG     8

#define CH_BIT      (1 << 7)
#define HOUR12_BIT  (1 << 6)
#define PM_BIT      (1 << 5)
#define SQWE_BIT    (1 << 4)
#define OUT_BIT     (1 << 7)

#define CH_MASK      0x7f
#define SECONDS_MASK 0x7f
#define HOUR12_MASK  0x1f
#define HOUR24_MASK  0x3f
#define SQWEF_MASK   0xfc
#define SQWE_MASK    0xef
#define OUT_MASK     0x7f

#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

static esp_err_t update_register(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    CHECK_ARG(dev);

    uint8_t old;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, &old, 1));
    uint8_t buf = (old & mask) | val;
    esp_err_t res = i2c_dev_write_reg(dev, reg, &buf, 1);
    I2C_DEV_GIVE_MUTEX(dev);

    return res;
}

esp_err_t ds1307_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = DS1307_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t ds1307_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t ds1307_start(i2c_dev_t *dev, bool start)
{
    return update_register(dev, TIME_REG, CH_MASK, start ? 0 : CH_BIT);
}

esp_err_t ds1307_is_running(i2c_dev_t *dev, bool *running)
{
    CHECK_ARG(dev && running);

    uint8_t val;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, TIME_REG, &val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *running = val & CH_BIT ? false : true;

    return ESP_OK;
}

esp_err_t ds1307_get_time(i2c_dev_t *dev, struct tm *time)
{
    CHECK_ARG(dev && time);

    uint8_t buf[7];

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, TIME_REG, buf, 7));
    I2C_DEV_GIVE_MUTEX(dev);

    time->tm_sec = bcd2dec(buf[0] & SECONDS_MASK);
    time->tm_min = bcd2dec(buf[1]);
    if (buf[2] & HOUR12_BIT)
    {
        // RTC in 12-hour mode
        time->tm_hour = bcd2dec(buf[2] & HOUR12_MASK) - 1;
        if (buf[2] & PM_BIT)
            time->tm_hour += 12;
    }
    else
        time->tm_hour = bcd2dec(buf[2] & HOUR24_MASK);
    time->tm_wday = bcd2dec(buf[3]) - 1;
    time->tm_mday = bcd2dec(buf[4]);
    time->tm_mon  = bcd2dec(buf[5]) - 1;
    time->tm_year = bcd2dec(buf[6]) + 100;

    return ESP_OK;
}

esp_err_t ds1307_set_time(i2c_dev_t *dev, const struct tm *time)
{
    CHECK_ARG(dev && time);

    uint8_t buf[7] = {
        dec2bcd(time->tm_sec),
        dec2bcd(time->tm_min),
        dec2bcd(time->tm_hour),
        dec2bcd(time->tm_wday + 1),
        dec2bcd(time->tm_mday),
        dec2bcd(time->tm_mon + 1),
        dec2bcd(time->tm_year - 100)
    };

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, TIME_REG, buf, sizeof(buf)));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ds1307_enable_squarewave(i2c_dev_t *dev, bool enable)
{
    return update_register(dev, CONTROL_REG, SQWE_MASK, enable ? SQWE_BIT : 0);
}

esp_err_t ds1307_is_squarewave_enabled(i2c_dev_t *dev, bool *sqw_en)
{
    CHECK_ARG(dev && sqw_en);

    uint8_t val;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, CONTROL_REG, &val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *sqw_en = val & SQWE_BIT;

    return ESP_OK;
}

esp_err_t ds1307_set_squarewave_freq(i2c_dev_t *dev, ds1307_squarewave_freq_t freq)
{
    return update_register(dev, CONTROL_REG, SQWEF_MASK, freq);
}

esp_err_t ds1307_get_squarewave_freq(i2c_dev_t *dev, ds1307_squarewave_freq_t *sqw_freq)
{
    CHECK_ARG(dev && sqw_freq);

    uint8_t val;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, CONTROL_REG, &val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *sqw_freq = val & ~SQWEF_MASK;

    return ESP_OK;
}

esp_err_t ds1307_get_output(i2c_dev_t *dev, bool *out)
{
    CHECK_ARG(dev && out);

    uint8_t val;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, CONTROL_REG, &val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *out = val & OUT_BIT;

    return ESP_OK;
}

esp_err_t ds1307_set_output(i2c_dev_t *dev, bool value)
{
    return update_register(dev, CONTROL_REG, OUT_MASK, value ? OUT_BIT : 0);
}

esp_err_t ds1307_read_ram(i2c_dev_t *dev, uint8_t offset, uint8_t *buf, uint8_t len)
{
    CHECK_ARG(dev && buf);

    if (offset + len > RAM_SIZE)
        return ESP_ERR_NO_MEM;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, RAM_REG + offset, buf, len));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ds1307_write_ram(i2c_dev_t *dev, uint8_t offset, uint8_t *buf, uint8_t len)
{
    CHECK_ARG(dev && buf);

    if (offset + len > RAM_SIZE)
        return ESP_ERR_NO_MEM;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, RAM_REG + offset, buf, len));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

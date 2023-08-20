/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file pcf8563.c
 *
 * ESP-IDF driver for PCF8563 (BM8563) real-time clock/calendar
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "pcf8563.h"

#define I2C_FREQ_HZ 400000

#define REG_CTRL_STATUS1 0x00
#define REG_CTRL_STATUS2 0x01
#define REG_VL_SECONDS   0x02
#define REG_MINUTES      0x03
#define REG_HOURS        0x04
#define REG_DAYS         0x05
#define REG_WEEKDAYS     0x06
#define REG_CENT_MONTHS  0x07
#define REG_YEARS        0x08
#define REG_ALARM_MIN    0x09
#define REG_ALARM_HOUR   0x0a
#define REG_ALARM_DAY    0x0b
#define REG_ALARM_WDAY   0x0c
#define REG_CLKOUT       0x0d
#define REG_TIMER_CTRL   0x0e
#define REG_TIMER        0x0f

#define BIT_YEAR_CENTURY 7
#define BIT_VL 7
#define BIT_AE 7
#define BIT_CLKOUT_FD 0
#define BIT_CLKOUT_FE 7

#define BIT_CTRL_STATUS2_TIE 0
#define BIT_CTRL_STATUS2_AIE 1
#define BIT_CTRL_STATUS2_TF  3
#define BIT_CTRL_STATUS2_AF  4

#define BIT_TIMER_CTRL_TE 7

#define MASK_TIMER_CTRL_TD 0x03
#define MASK_ALARM 0x7f

#define MASK_MIN  0x7f
#define MASK_HOUR 0x3f
#define MASK_MDAY 0x3f
#define MASK_WDAY 0x07
#define MASK_MON  0x1f

#define BV(x) ((uint8_t)(1 << (x)))

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

static inline esp_err_t read_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(dev, reg, val, 1);
}

static inline esp_err_t write_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(dev, reg, &val, 1);
}

static esp_err_t update_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t v;
    CHECK(read_reg_nolock(dev, reg, &v));
    CHECK(write_reg_nolock(dev, reg, (v & ~mask) | val));
    return ESP_OK;
}

static esp_err_t read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t update_reg(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_reg_nolock(dev, reg, mask, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t pcf8563_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = PCF8563_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t pcf8563_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t pcf8563_set_time(i2c_dev_t *dev, struct tm *time)
{
    CHECK_ARG(dev && time);

    bool ovf = time->tm_year >= 200;

    uint8_t data[7] = {
        dec2bcd(time->tm_sec),
        dec2bcd(time->tm_min),
        dec2bcd(time->tm_hour),
        dec2bcd(time->tm_mday),
        dec2bcd(time->tm_wday),
        dec2bcd(time->tm_mon + 1) | (ovf ? BV(BIT_YEAR_CENTURY) : 0),
        dec2bcd(time->tm_year - (ovf ? 200 : 100))
    };

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_VL_SECONDS, data, 7));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pcf8563_get_time(i2c_dev_t *dev, struct tm *time, bool *valid)
{
    CHECK_ARG(dev && time && valid);

    uint8_t data[7];

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_VL_SECONDS, data, 7));
    I2C_DEV_GIVE_MUTEX(dev);

    *valid = data[0] & BV(BIT_VL) ? false : true;
    time->tm_sec  = bcd2dec(data[0] & ~BV(BIT_VL));
    time->tm_min  = bcd2dec(data[1] & MASK_MIN);
    time->tm_hour = bcd2dec(data[2] & MASK_HOUR);
    time->tm_mday = bcd2dec(data[3] & MASK_MDAY);
    time->tm_wday = bcd2dec(data[4] & MASK_WDAY);
    time->tm_mon  = bcd2dec(data[5] & MASK_MON) - 1;
    time->tm_year = bcd2dec(data[6]) + (data[5] & BV(BIT_YEAR_CENTURY) ? 200 : 100);

    return ESP_OK;
}

esp_err_t pcf8563_set_clkout(i2c_dev_t *dev, pcf8563_clkout_freq_t freq)
{
    CHECK_ARG(dev);

    return write_reg(dev, REG_CLKOUT,
            freq == PCF8563_DISABLED
                ? 0
                : (BV(BIT_CLKOUT_FE) | ((freq - 1) & 3))
           );
}

esp_err_t pcf8563_get_clkout(i2c_dev_t *dev, pcf8563_clkout_freq_t *freq)
{
    CHECK_ARG(dev && freq);

    uint8_t v;
    CHECK(read_reg(dev, REG_CLKOUT, &v));
    *freq = v & BV(BIT_CLKOUT_FE) ? (v & 3) + 1 : PCF8563_DISABLED;

    return ESP_OK;
}

esp_err_t pcf8563_set_timer_settings(i2c_dev_t *dev, bool int_enable, pcf8563_timer_clock_t clock)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_reg_nolock(dev, REG_CTRL_STATUS2,
            BV(BIT_CTRL_STATUS2_TIE), int_enable ? BV(BIT_CTRL_STATUS2_TIE) : 0));
    I2C_DEV_CHECK(dev, update_reg_nolock(dev, REG_TIMER_CTRL, MASK_TIMER_CTRL_TD, clock));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pcf8563_get_timer_settings(i2c_dev_t *dev, bool *int_enabled, pcf8563_timer_clock_t *clock)
{
    CHECK_ARG(dev && int_enabled && clock);

    uint8_t s, t;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_nolock(dev, REG_CTRL_STATUS2, &s));
    I2C_DEV_CHECK(dev, read_reg_nolock(dev, REG_TIMER_CTRL, &t));
    I2C_DEV_GIVE_MUTEX(dev);

    *int_enabled = s & BV(BIT_CTRL_STATUS2_TIE) ? true : false;
    *clock = t & MASK_TIMER_CTRL_TD;

    return ESP_OK;
}

esp_err_t pcf8563_set_timer_value(i2c_dev_t *dev, uint8_t value)
{
    CHECK_ARG(dev);

    return write_reg(dev, REG_TIMER, value);
}

esp_err_t pcf8563_get_timer_value(i2c_dev_t *dev, uint8_t *value)
{
    CHECK_ARG(dev && value);

    return read_reg(dev, REG_TIMER, value);
}

esp_err_t pcf8563_start_timer(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return update_reg(dev, REG_TIMER_CTRL, BV(BIT_TIMER_CTRL_TE), BV(BIT_TIMER_CTRL_TE));
}

esp_err_t pcf8563_stop_timer(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return update_reg(dev, REG_TIMER_CTRL, BV(BIT_TIMER_CTRL_TE), 0);
}

esp_err_t pcf8563_get_timer_flag(i2c_dev_t *dev, bool *timer)
{
    CHECK_ARG(dev && timer);

    uint8_t v;
    CHECK(read_reg(dev, REG_CTRL_STATUS2, &v));
    *timer = v & BIT_CTRL_STATUS2_TF ? true : false;

    return ESP_OK;
}

esp_err_t pcf8563_clear_timer_flag(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return update_reg(dev, REG_CTRL_STATUS2, BV(BIT_CTRL_STATUS2_TF), 0);
}

esp_err_t pcf8563_set_alarm(i2c_dev_t *dev, bool int_enable, uint32_t flags, struct tm *time)
{
    CHECK_ARG(dev && time);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_reg_nolock(dev, REG_CTRL_STATUS2,
            BV(BIT_CTRL_STATUS2_AIE), int_enable ? BV(BIT_CTRL_STATUS2_AIE) : 0));
    uint8_t data[4] = {
        dec2bcd(time->tm_min) | (flags & PCF8563_ALARM_MATCH_MIN ? 0 : BV(BIT_AE)),
        dec2bcd(time->tm_hour) | (flags & PCF8563_ALARM_MATCH_HOUR ? 0 : BV(BIT_AE)),
        dec2bcd(time->tm_mday) | (flags & PCF8563_ALARM_MATCH_DAY ? 0 : BV(BIT_AE)),
        dec2bcd(time->tm_wday) | (flags & PCF8563_ALARM_MATCH_WEEKDAY ? 0 : BV(BIT_AE)),
    };
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_ALARM_MIN, data, 4));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pcf8563_get_alarm(i2c_dev_t *dev, bool *int_enabled, uint32_t *flags, struct tm *time)
{
    CHECK_ARG(dev && int_enabled && flags && time);

    uint8_t data[4], s;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_nolock(dev, REG_CTRL_STATUS2, &s));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_ALARM_MIN, data, 4));
    I2C_DEV_GIVE_MUTEX(dev);

    *int_enabled = s & BV(BIT_CTRL_STATUS2_AIE) ? true : false;
    *flags = 0;
    if (!(data[0] & BV(BIT_AE)))
        *flags |= PCF8563_ALARM_MATCH_MIN;
    if (!(data[1] & BV(BIT_AE)))
        *flags |= PCF8563_ALARM_MATCH_HOUR;
    if (!(data[2] & BV(BIT_AE)))
        *flags |= PCF8563_ALARM_MATCH_DAY;
    if (!(data[3] & BV(BIT_AE)))
        *flags |= PCF8563_ALARM_MATCH_WEEKDAY;

    time->tm_min = bcd2dec(data[0] & MASK_ALARM);
    time->tm_hour = bcd2dec(data[1] & MASK_ALARM);
    time->tm_mday = bcd2dec(data[2] & MASK_ALARM);
    time->tm_wday = bcd2dec(data[3] & MASK_ALARM);

    return ESP_OK;
}

esp_err_t pcf8563_get_alarm_flag(i2c_dev_t *dev, bool *alarm)
{
    CHECK_ARG(dev && alarm);

    uint8_t v;
    CHECK(read_reg(dev, REG_CTRL_STATUS2, &v));
    *alarm = v & BIT_CTRL_STATUS2_AF ? true : false;

    return ESP_OK;
}

esp_err_t pcf8563_clear_alarm_flag(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return update_reg(dev, REG_CTRL_STATUS2, BV(BIT_CTRL_STATUS2_AF), 0);
}

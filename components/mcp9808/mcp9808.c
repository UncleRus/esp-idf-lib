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
 * @file mcp9808.c
 *
 * ESP-IDF driver for MCP9808 Digital Temperature Sensor
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_idf_lib_helpers.h>
#include "mcp9808.h"

#include <math.h>
#include <esp_log.h>

static const char *TAG = "mcp9808";

#define I2C_FREQ_HZ 400000 // 400 kHz

#define MANUFACTURER_ID 0x0054
#define DEVICE_ID 0x04

#define REG_CONFIG  1
#define REG_T_UPPER 2
#define REG_T_LOWER 3
#define REG_T_CRIT  4
#define REG_T_A     5
#define REG_MANUF   6
#define REG_ID      7
#define REG_RES     8

#define BIT_CONFIG_HYST       9
#define BIT_CONFIG_SHDN       8
#define BIT_CONFIG_CRIT_LOCK  7
#define BIT_CONFIG_WIN_LOCK   6
#define BIT_CONFIG_INT_CLR    5
#define BIT_CONFIG_ALERT_STAT 4
#define BIT_CONFIG_ALERT_CTRL 3
#define BIT_CONFIG_ALERT_SEL  2
#define BIT_CONFIG_ALERT_POL  1
#define BIT_CONFIG_ALERT_MODE 0

#define BIT_T_SIGN 12

#define BIT_T_A_LOWER 13
#define BIT_T_A_UPPER 14
#define BIT_T_A_CRIT  15

#define BV(x) (1 << (x))

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static inline esp_err_t read_reg_16_nolock(i2c_dev_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK(i2c_dev_read_reg(dev, reg, val, 2));
    *val = (*val << 8) | (*val >> 8);
    return ESP_OK;
}

static inline esp_err_t write_reg_16_nolock(i2c_dev_t *dev, uint8_t reg, uint16_t val)
{
    uint16_t buf = (val << 8) | (val >> 8);
    return i2c_dev_write_reg(dev, reg, &buf, 2);
}

static esp_err_t read_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t *val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_16_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t write_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg_16_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t update_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t mask, uint16_t val)
{
    uint16_t old;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_16_nolock(dev, reg, &old));
    I2C_DEV_CHECK(dev, write_reg_16_nolock(dev, reg, (old & mask) | val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t write_temp(i2c_dev_t *dev, uint8_t reg, float t)
{
    uint16_t v;
    if (t < 0)
        v = (((1023 - ((uint16_t)(fabsf(t) * 4.0)) + 1) << 2) & 0x0fff) | BV(BIT_T_SIGN);
    else
        v = ((uint16_t)(t * 4.0) << 2) & 0x0fff;

    return write_reg_16(dev, reg, v);
}

static esp_err_t read_temp(i2c_dev_t *dev, uint8_t reg, float *t, uint16_t *raw)
{
    uint16_t v;
    CHECK(read_reg_16(dev, reg, &v));

    *t = (v & 0x0fff) / 16.0;
    if (v & BV(BIT_T_SIGN)) *t -= 256;

    if (raw) *raw = v;

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t mcp9808_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    // I2C Address verification is not needed because the device may have a custom factory address

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t mcp9808_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t mcp9808_init(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    uint16_t v;

    CHECK(read_reg_16(dev, REG_MANUF, &v));
    if (v != MANUFACTURER_ID)
    {
        ESP_LOGE(TAG, "Invalid manufacturer ID 0x%04x", v);
        return ESP_ERR_INVALID_RESPONSE;
    }

    CHECK(read_reg_16(dev, REG_ID, &v));
    if ((v >> 8) != DEVICE_ID)
    {
        ESP_LOGE(TAG, "Invalid device ID 0x%02x", v);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGD(TAG, "Device revision: 0x%02x", v & 0xff);

    // clear all lock bits
    return write_reg_16(dev, REG_CONFIG, 0);
}

esp_err_t mcp9808_set_mode(i2c_dev_t *dev, mcp9808_mode_t mode)
{
    CHECK_ARG(dev);

    return update_reg_16(dev, REG_CONFIG, ~BV(BIT_CONFIG_SHDN),
            mode == MCP9808_SHUTDOWN ? BV(BIT_CONFIG_SHDN) : 0);
}

esp_err_t mcp9808_get_mode(i2c_dev_t *dev, mcp9808_mode_t *mode)
{
    CHECK_ARG(dev && mode);

    uint16_t v;
    CHECK(read_reg_16(dev, REG_CONFIG, &v));
    *mode = v & BV(BIT_CONFIG_SHDN) ? MCP9808_SHUTDOWN : MCP9808_CONTINUOUS;

    return ESP_OK;
}

esp_err_t mcp9808_set_resolution(i2c_dev_t *dev, mcp9808_resolution_t res)
{
    CHECK_ARG(dev);

    return write_reg_16(dev, REG_RES, res);
}

esp_err_t mcp9808_get_resolution(i2c_dev_t *dev, mcp9808_resolution_t *res)
{
    CHECK_ARG(dev && res);

    return read_reg_16(dev, REG_RES, (uint16_t *)res);
}

esp_err_t mcp9808_set_alert_config(i2c_dev_t *dev, mcp9808_alert_mode_t mode,
        mcp9808_alert_select_t sel, mcp9808_alert_polarity_t polarity, mcp9808_hysteresis_t hyst)
{
    CHECK_ARG(dev);

    if (mode == MCP9808_ALERT_DISABLED)
        return update_reg_16(dev, REG_CONFIG, ~BV(BIT_CONFIG_ALERT_CTRL), 0);

    uint16_t mask = ~(BV(BIT_CONFIG_ALERT_CTRL) | BV(BIT_CONFIG_ALERT_SEL)
            | BV(BIT_CONFIG_ALERT_MODE) | BV(BIT_CONFIG_ALERT_POL) | (3 << BIT_CONFIG_HYST));

    return update_reg_16(dev, REG_CONFIG, mask,
            BV(BIT_CONFIG_ALERT_CTRL)
            | (mode == MCP9808_ALERT_COMPARATOR ? 0 : BV(BIT_CONFIG_ALERT_MODE))
            | ((sel & 1) << BIT_CONFIG_ALERT_SEL)
            | ((polarity & 1) << BIT_CONFIG_ALERT_POL)
            | ((hyst & 3) << BIT_CONFIG_HYST));
}

esp_err_t mcp9808_get_alert_config(i2c_dev_t *dev, mcp9808_alert_mode_t *mode,
        mcp9808_alert_select_t *sel, mcp9808_alert_polarity_t *polarity, mcp9808_hysteresis_t *hyst)
{
    CHECK_ARG(dev && mode && sel && polarity && hyst);

    uint16_t v;
    CHECK(read_reg_16(dev, REG_CONFIG, &v));

    if (v & BV(BIT_CONFIG_ALERT_CTRL))
        *mode = v & BV(BIT_CONFIG_ALERT_MODE) ? MCP9808_ALERT_INTERRUPT : MCP9808_ALERT_COMPARATOR;
    else
        *mode = MCP9808_ALERT_DISABLED;
    *sel = v & BV(BIT_CONFIG_ALERT_SEL) ? MCP9808_ALERT_CRIT : MCP9808_ALERT_UP_LOW_CRIT;
    *polarity = v & BV(BIT_CONFIG_ALERT_POL) ? MCP9808_ALERT_HIGH : MCP9808_ALERT_LOW;
    *hyst = v >> BIT_CONFIG_HYST;

    return ESP_OK;
}

esp_err_t mcp9808_set_limits(i2c_dev_t *dev, float t_upper, float t_lower, float t_crit)
{
    CHECK_ARG(dev);

    CHECK(write_temp(dev, REG_T_UPPER, t_upper));
    CHECK(write_temp(dev, REG_T_LOWER, t_lower));
    CHECK(write_temp(dev, REG_T_CRIT, t_crit));

    return ESP_OK;
}

esp_err_t mcp9808_get_limits(i2c_dev_t *dev, float *t_upper, float *t_lower, float *t_crit)
{
    CHECK_ARG(dev && t_upper && t_lower && t_crit);

    CHECK(read_temp(dev, REG_T_UPPER, t_upper, NULL));
    CHECK(read_temp(dev, REG_T_LOWER, t_lower, NULL));
    CHECK(read_temp(dev, REG_T_CRIT, t_crit, NULL));

    return ESP_OK;
}

esp_err_t mcp9808_set_alert_status(i2c_dev_t *dev, bool alert)
{
    CHECK_ARG(dev);

    return update_reg_16(dev, REG_CONFIG, ~BV(BIT_CONFIG_ALERT_STAT),
            alert ? BV(BIT_CONFIG_ALERT_STAT) : 0);
}

esp_err_t mcp9808_get_alert_status(i2c_dev_t *dev, bool *alert)
{
    CHECK_ARG(dev && alert);

    uint16_t v;
    CHECK(read_reg_16(dev, REG_CONFIG, &v));
    *alert = v & BV(BIT_CONFIG_ALERT_STAT) ? true : false;

    return ESP_OK;
}

esp_err_t mcp9808_clear_interrupt(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return update_reg_16(dev, REG_CONFIG, ~BV(BIT_CONFIG_INT_CLR), BV(BIT_CONFIG_INT_CLR));
}

esp_err_t mcp9808_get_temperature(i2c_dev_t *dev, float *t, bool *lower, bool *upper, bool *crit)
{
    CHECK_ARG(dev && t);

    uint16_t v;

    CHECK(read_temp(dev, REG_T_A, t, &v));
    if (lower) *lower = v & BV(BIT_T_A_LOWER) ? true : false;
    if (upper) *upper = v & BV(BIT_T_A_UPPER) ? true : false;
    if (crit) *crit = v & BV(BIT_T_A_CRIT) ? true : false;

    return ESP_OK;
}


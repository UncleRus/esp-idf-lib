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
 * @file bh1900nux.c
 *
 * ESP-IDF driver for BH1900NUX temperature sensor
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include "bh1900nux.h"

static const char *TAG = "bh1900nux";

static const float t_lsb = 0.0625;

#define BH1900NUX_I2C_ADDR_MAX 0x4f

#define I2C_FREQ_HZ 400000 // 400kHz

#define REG_TEMP  0
#define REG_CONF  1
#define REG_TLOW  2
#define REG_THIGH 3
#define REG_RST   4

#define BIT_SHUTDOWN  0
#define BIT_ALERT_POL 2
#define BIT_FAULT_DQ0 3
#define BIT_ALERT     6
#define BIT_ONE_SHOT  7
#define BIT_WT0       8

#define CONV_TIME_MS 35
#define RESET_TIME_MS 250

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

inline static uint16_t shuffle(uint16_t v)
{
    return (v << 8) | (v >> 8);
}

static esp_err_t read_reg(bh1900nux_t *dev, uint8_t reg, uint16_t *val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, val, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val = shuffle(*val);

    return ESP_OK;
}

static esp_err_t write_reg(bh1900nux_t *dev, uint8_t reg, uint16_t val)
{
    uint16_t v = shuffle(val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg, &v, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_temp(bh1900nux_t *dev, uint8_t reg, float *temp)
{
    CHECK_ARG(dev && temp);

    int16_t buf;

    CHECK(read_reg(dev, reg, (uint16_t *)&buf));

    *temp = ((int16_t)buf >> 4) * t_lsb;

    return ESP_OK;
}

static esp_err_t write_temp(bh1900nux_t *dev, uint8_t reg, float temp)
{
    CHECK_ARG(dev);

    return write_reg(dev, reg, (int16_t)(temp / t_lsb) << 4);
}

static esp_err_t update_reg(bh1900nux_t *dev, uint8_t reg, uint16_t val, uint16_t mask)
{
    CHECK_ARG(dev);

    uint16_t tmp;
    CHECK(read_reg(dev, reg, &tmp));
    CHECK(write_reg(dev, reg, (tmp & ~mask) | (val & ~mask)));

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t bh1900nux_init_desc(bh1900nux_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr < BH1900NUX_I2C_ADDR_BASE || addr > BH1900NUX_I2C_ADDR_MAX)
    {
        ESP_LOGE(TAG, "Invalid device address: 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t bh1900nux_free_desc(bh1900nux_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bh1900nux_reset(bh1900nux_t *dev)
{
    CHECK_ARG(dev);

    CHECK(write_reg(dev, REG_RST, 1));
    vTaskDelay(pdMS_TO_TICKS(RESET_TIME_MS));
    return bh1900nux_set_mode(dev, dev->mode);
}

esp_err_t bh1900nux_set_mode(bh1900nux_t *dev, bh1900nux_mode_t mode)
{
    CHECK_ARG(dev);

    dev->mode = mode;

    uint16_t val, mask;
    if (mode == BH1900NUX_MODE_CONTINUOUS)
    {
        val = BV(BIT_ONE_SHOT);
        mask = BV(BIT_ONE_SHOT) | BV(BIT_SHUTDOWN);
    }
    else
    {
        val = BV(BIT_SHUTDOWN);
        mask = BV(BIT_ONE_SHOT) | BV(BIT_SHUTDOWN);
    }

    return update_reg(dev, REG_CONF, val, mask);
}

esp_err_t bh1900nux_get_config(bh1900nux_t *dev, bh1900nux_wait_time_t *wt, bh1900nux_fault_queue_t *fq, bh1900nux_alert_polarity_t *ap)
{
    CHECK_ARG(dev && wt && fq && ap);

    uint16_t val;
    CHECK(read_reg(dev, REG_CONF, &val));

    *wt = (val >> BIT_WT0) & 3;
    *fq = (val >> BIT_FAULT_DQ0) & 3;
    *ap = (val >> BIT_ALERT_POL) & 1;

    return ESP_OK;
}

esp_err_t bh1900nux_set_config(bh1900nux_t *dev, bh1900nux_wait_time_t wt, bh1900nux_fault_queue_t fq, bh1900nux_alert_polarity_t ap)
{
    CHECK_ARG(dev && fq <= BH1900NUX_FAULTS_6 && wt <= BH1900NUX_WT_3);

    uint16_t val = (wt << BIT_WT0) | (fq << BIT_FAULT_DQ0) | (ap != BH1900NUX_ALERT_LOW ? BV(BIT_ALERT_POL) : 0);

    CHECK(write_reg(dev, REG_CONF, val));
    CHECK(bh1900nux_set_mode(dev, dev->mode));

    return ESP_OK;
}

esp_err_t bh1900nux_one_shot(bh1900nux_t *dev, float *temp)
{
    CHECK_ARG(dev);

    if (dev->mode != BH1900NUX_MODE_SHUTDOWN)
    {
        ESP_LOGE(TAG, "Cannot perform one-shot measurement, device in invalid mode");
        return ESP_ERR_INVALID_STATE;
    }

    CHECK(update_reg(dev, REG_CONF, BV(BIT_ONE_SHOT) | BV(BIT_SHUTDOWN), BV(BIT_ONE_SHOT) | BV(BIT_SHUTDOWN)));

    vTaskDelay(pdMS_TO_TICKS(CONV_TIME_MS));

    return read_temp(dev, REG_TEMP, temp);
}

esp_err_t bh1900nux_get_temperature(bh1900nux_t *dev, float *temp)
{
    return read_temp(dev, REG_TEMP, temp);
}

esp_err_t bh1900nux_get_t_low(bh1900nux_t *dev, float *temp)
{
    return read_temp(dev, REG_TLOW, temp);
}

esp_err_t bh1900nux_set_t_low(bh1900nux_t *dev, float temp)
{
    return write_temp(dev, REG_TLOW, temp);
}

esp_err_t bh1900nux_get_t_high(bh1900nux_t *dev, float *temp)
{
    return read_temp(dev, REG_THIGH, temp);
}

esp_err_t bh1900nux_set_t_high(bh1900nux_t *dev, float temp)
{
    return write_temp(dev, REG_THIGH, temp);
}

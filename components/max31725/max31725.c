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
 * @file max31725.c
 *
 * ESP-IDF driver for MAX31725/MAX31726 temperature sensors
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include "max31725.h"

static const char *TAG = "max31725";

static const float t_lsb = 0.00390625;

#define MAX31725_I2C_ADDR_MAX 0x5f

#define I2C_FREQ_HZ 400000 // 400kHz

#define REG_TEMP 0
#define REG_CONF 1
#define REG_HYST 2
#define REG_OS   3

#define BIT_SHUTDOWN 0
#define BIT_COMP_INT 1
#define BIT_OS_POL   2
#define BIT_FAULT_Q0 3
#define BIT_DATA_FMT 5
#define BIT_TIMEOUT  6
#define BIT_ONE_SHOT 7

#define FMT_SHIFT 64.0
#define CONV_TIME_MS 50

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

typedef union
{
    uint16_t udata;
    int16_t sdata;
} temp_data_t;

static esp_err_t read_temp(i2c_dev_t *dev, uint8_t reg, float *temp, max31725_data_format_t fmt)
{
    CHECK_ARG(dev && temp);

    temp_data_t buf;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, &buf.udata, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    buf.udata = (buf.udata << 8) | (buf.udata >> 8);
    *temp = buf.sdata * t_lsb + (fmt == MAX31725_FMT_EXTENDED ? FMT_SHIFT : 0);

    return ESP_OK;
}

static esp_err_t write_temp(i2c_dev_t *dev, uint8_t reg, float temp, max31725_data_format_t fmt)
{
    CHECK_ARG(dev);

    temp_data_t buf;
    buf.sdata = (temp - (fmt == MAX31725_FMT_EXTENDED ? FMT_SHIFT : 0)) / t_lsb;
    buf.udata = (buf.udata << 8) | (buf.udata >> 8);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &buf.udata, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t max31725_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr < MAX31725_I2C_ADDR_BASE || addr > MAX31725_I2C_ADDR_MAX)
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

esp_err_t max31725_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t max31725_get_config(i2c_dev_t *dev, max31725_mode_t *mode, max31725_data_format_t *fmt, max31725_fault_queue_t *fq,
        max31725_os_polarity_t *op, max31725_os_mode_t *om)
{
    CHECK_ARG(dev && mode && fmt && fq && op && om);

    uint8_t b;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_CONF, &b, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *mode = (b >> BIT_SHUTDOWN) & 1;
    *fmt = (b >> BIT_DATA_FMT) & 1;
    *fq = (b >> BIT_FAULT_Q0) & 3;
    *op = (b >> BIT_OS_POL) & 1;
    *om = (b >> BIT_COMP_INT) & 1;

    return ESP_OK;
}

esp_err_t max31725_set_config(i2c_dev_t *dev, max31725_mode_t mode, max31725_data_format_t fmt, max31725_fault_queue_t fq,
        max31725_os_polarity_t op, max31725_os_mode_t om)
{
    CHECK_ARG(dev && fq <= MAX31725_FAULTS_6);

    uint8_t b = (fmt != MAX31725_FMT_NORMAL ? BV(BIT_DATA_FMT) : 0) |
                (fq << BIT_FAULT_Q0) |
                (op != MAX31725_OS_LOW ? BV(BIT_OS_POL) : 0) |
                (om != MAX31725_OS_COMPARATOR ? BV(BIT_COMP_INT) : 0) |
                (mode != MAX31725_MODE_CONTINUOUS ? BV(BIT_SHUTDOWN) : 0);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_CONF, &b, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t max31725_one_shot(i2c_dev_t *dev, float *temp, max31725_data_format_t fmt)
{
    CHECK_ARG(dev);

    uint8_t b;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_CONF, &b, 1));
    b |= BV(BIT_ONE_SHOT);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_CONF, &b, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    // wait 50 ms
    vTaskDelay(pdMS_TO_TICKS(CONV_TIME_MS));

    return read_temp(dev, REG_TEMP, temp, fmt);
}

esp_err_t max31725_get_temperature(i2c_dev_t *dev, float *temp, max31725_data_format_t fmt)
{
    return read_temp(dev, REG_TEMP, temp, fmt);
}

esp_err_t max31725_get_os_temp(i2c_dev_t *dev, float *temp, max31725_data_format_t fmt)
{
    return read_temp(dev, REG_OS, temp, fmt);
}

esp_err_t max31725_set_os_temp(i2c_dev_t *dev, float temp, max31725_data_format_t fmt)
{
    return write_temp(dev, REG_OS, temp, fmt);
}

esp_err_t max31725_get_hysteresis_temp(i2c_dev_t *dev, float *temp, max31725_data_format_t fmt)
{
    return read_temp(dev, REG_HYST, temp, fmt);
}

esp_err_t max31725_set_hysteresis_temp(i2c_dev_t *dev, float temp, max31725_data_format_t fmt)
{
    return write_temp(dev, REG_HYST, temp, fmt);
}

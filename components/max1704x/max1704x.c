/*
 * Copyright (c) 2022 Joshua Butler, MD, MHI <josh.butler929@gmail.com>
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
 * @file max1704x.c
 *
 * ESP-IDF driver for MAX17043/MAX17044/MAX17048/MAX17049 battery fuel gauge
 * 
 * Copyright (c) 2022 Joshua Butler, MD, MHI <josh.butler929@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

#include "max1704x.h"

#define I2C_FREQ_HZ 400000

/**
 * MAX1704X registers
 */
#define MAX1704X_REGISTER_VCELL         0x02
#define MAX1704X_REGISTER_SOC           0x04
#define MAX1704X_REGISTER_MODE          0x06
#define MAX1704X_REGISTER_VERSION       0x08
#define MAX1704X_REGISTER_HIBRT         0x0A
#define MAX1704X_REGISTER_CONFIG        0x0C
#define MAX1704X_REGISTER_VALRT         0x14    // MAX17048/MAX17049 only
#define MAX1704X_REGISTER_CRATE         0x16    // MAX17048/MAX17049 only
#define MAX1704X_REGISTER_VRESET        0x18    // MAX17048/MAX17049 only
#define MAX1704X_REGISTER_STATUS        0x1A    // MAX17048/MAX17049 only
#define MAX1704X_REGISTER_COMMAND       0xFE

/**
 * MAX1704X modes
 */

#define MAX1704X_RESET_COMMAND          0x5400
#define MAX1704X_QUICKSTART_MODE        0x4000

/**
 * MAX1704X config register bits
 */

#define MAX1704X_CONFIG_ALRT_BIT        ((1U << 5))
#define MAX1704X_CONFIG_ALSC_BIT        ((1U << 6))
#define MAX1704X_CONFIG_SLEEP_BIT       ((1U << 7))
#define MAX1704X_CONFIG_ATHD_MASK       0x1F
#define MAX1704X_CONFIG_ATHD_SHIFT      0
#define MAX1704X_STATUS_RI_BIT          ((1U << 0))
#define MAX1704X_STATUS_VH_BIT          ((1U << 1))
#define MAX1704X_STATUS_VL_BIT          ((1U << 2))
#define MAX1704X_STATUS_VR_BIT          ((1U << 3))
#define MAX1704X_STATUS_SL_BIT          ((1U << 4))
#define MAX1704X_STATUS_SC_BIT          ((1U << 5))
#define MAX1704X_STATUS_VRA_BIT         ((1U << 6))

/**
 * MAX1704X precision constants:
 * 
 * Precision calculated is by per bit, not per LSB.
 */
 
#define MAX17043_MV_PRECISION           1.25f        // 1.25mV precision
#define MAX17048_MV_PRECISION           0.078125f    // 0.078125mV precision
#define MAX1704X_CRATE_PRECISION        0.208f       // 0.208% precision
#define MAX1704X_VALRT_PRECISION        20.0f        // 20mV precision
#define MAX1704X_HIBRT_VCELL_PRECISION  1.25f        // 1.25mV precision
#define MAX1704X_HIBRT_CRATE_PRECISION  0.208f       // 0.208% precision

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static char *TAG = "max1704x";

/**
 * Private functions
 */

int16_t be16_to_cpu_signed(const uint8_t data[2])
{
    int16_t r;
    uint16_t u = (unsigned)data[1] | ((unsigned)data[0] << 8);
    memcpy(&r, &u, sizeof r);
    return r;
}

int16_t le16_to_cpu_signed(const uint8_t data[2])
{
    int16_t r;
    uint16_t u = (unsigned)data[0] | ((unsigned)data[1] << 8);
    memcpy(&r, &u, sizeof r);
    return r;
}

/**
 * Public functions
 */

esp_err_t max1704x_init_desc(max1704x_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = MAX1704X_I2C_ADDR;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t max1704x_free_desc(max1704x_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t max1704x_quickstart(max1704x_t *dev)
{
    CHECK_ARG(dev);

    uint8_t data[2] = { 0x40, 0x00 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, MAX1704X_REGISTER_MODE, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "MAX1704X Quickstart");

    return ESP_OK;
}

esp_err_t max1704x_get_voltage(max1704x_t *dev, float *voltage)
{
    CHECK_ARG(dev && voltage);

    uint8_t data[2];
    int value;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MAX1704X_REGISTER_VCELL, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    // ESP_LOG_BUFFER_HEXDUMP("voltage", data, 2, ESP_LOG_INFO);

    if (dev->model == MAX17043_4) {
        value = (data[0] << 4) | (data[1] >> 4);
        *voltage = ((float)value * MAX17043_MV_PRECISION) / 1000;
    } else {
        *voltage = (((float)data[0] * 256 + (float)data[1]) * MAX17048_MV_PRECISION) / 1000;
    }
    return ESP_OK;
}

esp_err_t max1704x_get_soc(max1704x_t *dev, float *soc)
{
    CHECK_ARG(dev && soc);

    uint8_t data[2];
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MAX1704X_REGISTER_SOC, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    // ESP_LOG_BUFFER_HEXDUMP("soc", data, 2, ESP_LOG_INFO);
    
    *soc = (float)data[0] + ((float)data[1]) / 256;
    if (*soc > 100) {
        *soc = 100.0f;
    }
    return ESP_OK;
}

esp_err_t max1704x_get_crate(max1704x_t *dev, float *crate)
{
    CHECK_ARG(dev && crate);

    int16_t crate_value;
    uint8_t data[2];
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MAX1704X_REGISTER_CRATE, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    
    if (dev->model == MAX17043_4) {
        ESP_LOGE(TAG, "MAX1704X_REGISTER_CRATE is not supported by MAX17043");
    } else {
        crate_value = be16_to_cpu_signed(data);
        ESP_LOGD(TAG, "crateValue: %d", (int)crate_value);
        *crate = ((float)crate_value * MAX1704X_CRATE_PRECISION);
    }
    return ESP_OK;
}

esp_err_t max1704x_get_version(max1704x_t *dev, uint16_t *version)
{
    CHECK_ARG(dev && version);

    uint8_t data[2];
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MAX1704X_REGISTER_VERSION, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    
    *version = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t max1704x_get_config(max1704x_t *dev)
{
    CHECK_ARG(dev);

    uint8_t data[2];
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MAX1704X_REGISTER_CONFIG, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->config.rcomp = data[0];
    dev->config.sleep_mode = (data[1] & MAX1704X_CONFIG_SLEEP_BIT) ? true : false;
    dev->config.soc_change_alert = (data[1] & MAX1704X_CONFIG_ALSC_BIT) ? true : false;
    dev->config.alert_status = (data[1] & MAX1704X_CONFIG_ALRT_BIT) ? true : false;
    dev->config.empty_alert_thresh = 32 - ((data[1] & MAX1704X_CONFIG_ATHD_MASK) >> MAX1704X_CONFIG_ATHD_SHIFT);

    return ESP_OK;
}

esp_err_t max1704x_set_config(max1704x_t *dev, max1704x_config_t *config)
{
    CHECK_ARG(dev && config);

    uint8_t data[2];
    
    if (config->rcomp) {
       data[0] = config->rcomp;
       dev->config.rcomp = config->rcomp;
    }

    data[1] = 0;
    dev->config.sleep_mode = config->sleep_mode;
    if (config->sleep_mode) {
        data[1] |= MAX1704X_CONFIG_SLEEP_BIT;
    }

    dev->config.soc_change_alert = config->soc_change_alert;
    if (config->soc_change_alert) {
        data[1] |= MAX1704X_CONFIG_ALSC_BIT;
    }

    dev->config.alert_status = config->alert_status;
    if (config->alert_status) {
        data[1] |= MAX1704X_CONFIG_ALRT_BIT;
    }

    dev->config.empty_alert_thresh = 32 - config->empty_alert_thresh;
    data[1] |= (32 - config->empty_alert_thresh) << MAX1704X_CONFIG_ATHD_SHIFT;
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, MAX1704X_REGISTER_CONFIG, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    
    return ESP_OK;
}

esp_err_t max1704x_get_status(max1704x_t *dev)
{
    CHECK_ARG(dev);

    uint8_t data[2];
    
    if (dev->model == MAX17043_4) {
        ESP_LOGE(TAG, "MAX1704X STATUS is not supported by MAX17043");
        return ESP_ERR_NOT_SUPPORTED;
    }

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MAX1704X_REGISTER_STATUS, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->status.reset_indicator = (data[0] & MAX1704X_STATUS_RI_BIT) ? true : false;
    dev->status.voltage_high = (data[0] & MAX1704X_STATUS_VH_BIT) ? true : false;
    dev->status.voltage_low = (data[0] & MAX1704X_STATUS_VL_BIT) ? true : false;
    dev->status.voltage_reset = (data[0] & MAX1704X_STATUS_VR_BIT) ? true : false; 
    dev->status.soc_low = (data[0] & MAX1704X_STATUS_SL_BIT) ? true : false;
    dev->status.soc_change = (data[0] & MAX1704X_STATUS_SC_BIT) ? true : false;
    dev->status.vreset_alert = (data[0] & MAX1704X_STATUS_VRA_BIT) ? true : false;

    return ESP_OK;
}

esp_err_t max1704x_set_status(max1704x_t *dev, max1704x_status_t *status)
{
    CHECK_ARG(dev && status);

    uint8_t data[2];
    
    if (dev->model == MAX17043_4) {
        ESP_LOGE(TAG, "MAX1704X STATUS is not supported by MAX17043");
        return ESP_ERR_NOT_SUPPORTED;
    }

    data[0] = 0;
    if (status->reset_indicator) {
        data[0] |= MAX1704X_STATUS_RI_BIT;
    }
    if (status->voltage_high) {
        data[0] |= MAX1704X_STATUS_VH_BIT;
    }
    if (status->voltage_low) {
        data[0] |= MAX1704X_STATUS_VL_BIT;
    }
    if (status->voltage_reset) {
        data[0] |= MAX1704X_STATUS_VR_BIT;
    }
    if (status->soc_low) {
        data[0] |= MAX1704X_STATUS_SL_BIT;
    }
    if (status->soc_change) {
        data[0] |= MAX1704X_STATUS_SC_BIT;
    }
    if (status->vreset_alert) {
        data[0] |= MAX1704X_STATUS_VRA_BIT;
    }
    data[1] = 0;
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, MAX1704X_REGISTER_STATUS, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

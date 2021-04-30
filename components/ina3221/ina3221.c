/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Zaltora <https://github.com/Zaltora>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ina3221.c
 *
 * ESP-IDF driver for Shunt and Bus Voltage Monitor INA3221
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Zaltora <https://github.com/Zaltora>\n
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "ina3221.h"

static const char *TAG = "ina3221";

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 2.44Mhz

#define INA3221_REG_CONFIG                      (0x00)
#define INA3221_REG_SHUNTVOLTAGE_1              (0x01)
#define INA3221_REG_BUSVOLTAGE_1                (0x02)
#define INA3221_REG_CRITICAL_ALERT_1            (0x07)
#define INA3221_REG_WARNING_ALERT_1             (0x08)
#define INA3221_REG_SHUNT_VOLTAGE_SUM           (0x0D)
#define INA3221_REG_SHUNT_VOLTAGE_SUM_LIMIT     (0x0E)
#define INA3221_REG_MASK                        (0x0F)
#define INA3221_REG_VALID_POWER_UPPER_LIMIT     (0x10)
#define INA3221_REG_VALID_POWER_LOWER_LIMIT     (0x11)

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t read_reg_16(ina3221_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK_ARG(val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, val, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val = (*val >> 8) | (*val << 8);  // Swap

    return ESP_OK;
}

static esp_err_t write_reg_16(ina3221_t *dev, uint8_t reg, uint16_t val)
{
    uint16_t v = (val >> 8) | (val << 8);  // Swap

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg, &v, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t write_config(ina3221_t *dev)
{
    return write_reg_16(dev, INA3221_REG_CONFIG, dev->config.config_register);
}

static esp_err_t write_mask(ina3221_t *dev)
{
    return write_reg_16(dev, INA3221_REG_MASK, dev->mask.mask_register & INA3221_MASK_CONFIG);
}

///////////////////////////////////////////////////////////////////////////////////

esp_err_t ina3221_init_desc(ina3221_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr < INA3221_I2C_ADDR_GND || addr > INA3221_I2C_ADDR_SCL)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
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

esp_err_t ina3221_free_desc(ina3221_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}


esp_err_t ina3221_sync(ina3221_t *dev)
{
    CHECK_ARG(dev);

    uint16_t data;

    // Sync config register
    CHECK(read_reg_16(dev, INA3221_REG_CONFIG, &data));
    if (data != dev->config.config_register)
        CHECK(write_config(dev));

    // Sync mask register
    CHECK(read_reg_16(dev, INA3221_REG_MASK, &data));
    if ((data & INA3221_MASK_CONFIG) != (dev->mask.mask_register & INA3221_MASK_CONFIG))
        CHECK(write_mask(dev));

    return ESP_OK;
}

esp_err_t ina3221_trigger(ina3221_t *dev)
{
    return write_config(dev);
}

esp_err_t ina3221_get_status(ina3221_t *dev)
{
    return read_reg_16(dev, INA3221_REG_MASK, &dev->mask.mask_register);
}

esp_err_t ina3221_set_options(ina3221_t *dev, bool mode, bool bus, bool shunt)
{
    CHECK_ARG(dev);

    dev->config.mode = mode;
    dev->config.ebus = bus;
    dev->config.esht = shunt;
    return write_config(dev);
}

esp_err_t ina3221_enable_channel(ina3221_t *dev, bool ch1, bool ch2, bool ch3)
{
    CHECK_ARG(dev);

    dev->config.ch1 = ch1;
    dev->config.ch2 = ch2;
    dev->config.ch3 = ch3;
    return write_config(dev);
}

esp_err_t ina3221_enable_channel_sum(ina3221_t *dev, bool ch1, bool ch2, bool ch3)
{
    CHECK_ARG(dev);

    dev->mask.scc1 = ch1;
    dev->mask.scc2 = ch2;
    dev->mask.scc3 = ch3;
    return write_mask(dev);
}

esp_err_t ina3221_enable_latch_pin(ina3221_t *dev, bool warning, bool critical)
{
    CHECK_ARG(dev);

    dev->mask.wen = warning;
    dev->mask.cen = critical;
    return write_mask(dev);
}

esp_err_t ina3221_set_average(ina3221_t *dev, ina3221_avg_t avg)
{
    CHECK_ARG(dev);

    dev->config.avg = avg;
    return write_config(dev);
}

esp_err_t ina3221_set_bus_conversion_time(ina3221_t *dev, ina3221_ct_t ct)
{
    CHECK_ARG(dev);

    dev->config.vbus = ct;
    return write_config(dev);
}

esp_err_t ina3221_set_shunt_conversion_time(ina3221_t *dev, ina3221_ct_t ct)
{
    CHECK_ARG(dev);

    dev->config.vsht = ct;
    return write_config(dev);
}

esp_err_t ina3221_reset(ina3221_t *dev)
{
    CHECK_ARG(dev);

    dev->config.config_register = INA3221_DEFAULT_CONFIG;
    dev->mask.mask_register = INA3221_DEFAULT_CONFIG;
    dev->config.rst = 1;
    return write_config(dev);
}

esp_err_t ina3221_get_bus_voltage(ina3221_t *dev, ina3221_channel_t channel, float *voltage)
{
    CHECK_ARG(dev && voltage);

    int16_t raw;

    CHECK(read_reg_16(dev, INA3221_REG_BUSVOLTAGE_1 + channel * 2, (uint16_t *)&raw));
    *voltage = raw * 0.001;

    return ESP_OK;
}

esp_err_t ina3221_get_shunt_value(ina3221_t *dev, ina3221_channel_t channel, float *voltage, float *current)
{
    CHECK_ARG(dev);
    CHECK_ARG(voltage || current);
    if (current && !dev->shunt[channel])
    {
        ESP_LOGE(TAG, "No shunt configured for channel %u in device [0x%02x at %d]", channel, dev->i2c_dev.addr, dev->i2c_dev.port);
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw;
    CHECK(read_reg_16(dev, INA3221_REG_SHUNTVOLTAGE_1 + channel * 2, (uint16_t *)&raw));
    float mvolts = raw * 0.005; // mV, 40uV step

    if (voltage)
        *voltage = mvolts;

    if (current)
        *current = mvolts * 1000.0 / dev->shunt[channel];  // mA

    return ESP_OK;
}

esp_err_t ina3221_get_sum_shunt_value(ina3221_t *dev, float *voltage)
{
    CHECK_ARG(dev && voltage);

    int16_t raw;

    CHECK(read_reg_16(dev, INA3221_REG_SHUNT_VOLTAGE_SUM, (uint16_t *)&raw));
    *voltage = raw * 0.02; // mV

    return ESP_OK;
}

esp_err_t ina3221_set_critical_alert(ina3221_t *dev, ina3221_channel_t channel, float current)
{
    CHECK_ARG(dev);

    int16_t raw = current * dev->shunt[channel] * 0.2;
    return write_reg_16(dev, INA3221_REG_CRITICAL_ALERT_1 + channel * 2, *(uint16_t *)&raw);
}

esp_err_t ina3221_set_warning_alert(ina3221_t *dev, ina3221_channel_t channel, float current)
{
    CHECK_ARG(dev);

    int16_t raw = current * dev->shunt[channel] * 0.2;
    return write_reg_16(dev, INA3221_REG_WARNING_ALERT_1 + channel * 2, *(uint16_t *)&raw);
}

esp_err_t ina3221_set_sum_warning_alert(ina3221_t *dev, float voltage)
{
    CHECK_ARG(dev);

    int16_t raw = voltage * 50.0;
    return write_reg_16(dev, INA3221_REG_SHUNT_VOLTAGE_SUM_LIMIT, *(uint16_t *)&raw);
}

esp_err_t ina3221_set_power_valid_upper_limit(ina3221_t *dev, float voltage)
{
    CHECK_ARG(dev);
    if (!dev->config.ebus)
    {
        ESP_LOGE(TAG, "Bus is not enabled in device [0x%02x at %d]", dev->i2c_dev.addr, dev->i2c_dev.port);
        return ESP_ERR_NOT_SUPPORTED;
    }

    int16_t raw = voltage * 1000.0;
    return write_reg_16(dev, INA3221_REG_VALID_POWER_UPPER_LIMIT, *(uint16_t *)&raw);
}

esp_err_t ina3221_set_power_valid_lower_limit(ina3221_t *dev, float voltage)
{
    CHECK_ARG(dev);
    if (!dev->config.ebus)
    {
        ESP_LOGE(TAG, "Bus is not enabled in device [0x%02x at %d]", dev->i2c_dev.addr, dev->i2c_dev.port);
        return ESP_ERR_NOT_SUPPORTED;
    }

    int16_t raw = voltage * 1000.0;
    return write_reg_16(dev, INA3221_REG_VALID_POWER_LOWER_LIMIT, *(uint16_t *)&raw);
}

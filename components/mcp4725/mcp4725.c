/**
 * @file mcp4725.c
 *
 * ESP-IDF Driver for 12-bit DAC MCP4725
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016, 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "mcp4725.h"

static const char *TAG = "MCP4725";

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 3.4Mhz

#define CMD_DAC    0x40
#define CMD_EEPROM 0x60
#define BIT_READY  0x80

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t read_data(i2c_dev_t *dev, void *data, uint8_t size)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, data, size));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mcp4725_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr < MCP4725A0_I2C_ADDR0 || addr > MCP4725A2_I2C_ADDR1)
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

esp_err_t mcp4725_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t mcp4725_eeprom_busy(i2c_dev_t *dev, bool *busy)
{
    CHECK_ARG(dev && busy);

    uint8_t res;
    CHECK(read_data(dev, &res, 1));

    *busy = !(res & BIT_READY);

    return ESP_OK;
}

esp_err_t mcp4725_get_power_mode(i2c_dev_t *dev, bool eeprom, mcp4725_power_mode_t *mode)
{
    CHECK_ARG(dev && mode);

    uint8_t buf[4];
    CHECK(read_data(dev, buf, eeprom ? 4 : 1));

    *mode = (eeprom ? buf[3] >> 5 : buf[0] >> 1) & 0x03;

    return ESP_OK;
}

esp_err_t mcp4725_set_power_mode(i2c_dev_t *dev, bool eeprom, mcp4725_power_mode_t mode)
{
    CHECK_ARG(dev);

    uint16_t value;
    CHECK(mcp4725_get_raw_output(dev, eeprom, &value));

    uint8_t data[] = {
        (eeprom ? CMD_EEPROM : CMD_DAC) | (((uint8_t)mode & 3) << 1),
        value >> 4,
        value << 4
    };

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, data, 3));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mcp4725_get_raw_output(i2c_dev_t *dev, bool eeprom, uint16_t *value)
{
    CHECK_ARG(dev && value);

    uint8_t buf[5];
    CHECK(read_data(dev, buf, eeprom ? 5 : 3));

    *value = eeprom
        ? ((uint16_t)(buf[3] & 0x0f) << 8) | buf[4]
        : ((uint16_t)buf[0] << 4) | (buf[1] >> 4);

    return ESP_OK;
}

esp_err_t mcp4725_set_raw_output(i2c_dev_t *dev, uint16_t value, bool eeprom)
{
    CHECK_ARG(dev);

    uint8_t data[] = {
        (eeprom ? CMD_EEPROM : CMD_DAC),
        value >> 4,
        value << 4
    };

    ESP_LOGV(TAG, "Set output value to %u", value);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, data, 3));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mcp4725_get_voltage(i2c_dev_t *dev, float vdd, bool eeprom, float *voltage)
{
    CHECK_ARG(voltage);

    uint16_t value;
    CHECK(mcp4725_get_raw_output(dev, eeprom, &value));

    *voltage = vdd / MCP4725_MAX_VALUE * value;

    return ESP_OK;
}

esp_err_t mcp4725_set_voltage(i2c_dev_t *dev, float vdd, float value, bool eeprom)
{
    return mcp4725_set_raw_output(dev, MCP4725_MAX_VALUE / vdd * value, eeprom);
}


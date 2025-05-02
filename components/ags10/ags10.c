/**
 * @file ags10.c
 * @brief AGS10 sensor driver implementation
 * @author xyzroe
 * ESP-IDF driver for AGS10 sensor
 *
 * Copyright (c) 2024 xyzroe <i@xyzroe.cc>
 * Licensed as described in the file LICENSE
 */

#include "ags10.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/i2c.h>

#define CHECK_ARG(VAL)                                                                                                                                                                                 \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (!(VAL))                                                                                                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                                                                                                \
    }                                                                                                                                                                                                  \
    while (0)

static const char *TAG = "ags10";

static uint8_t crc8(const uint8_t *data, int len);
static bool _read_reg(i2c_dev_t *dev, uint8_t cmd, uint16_t delayms, uint8_t *value, size_t value_size);

esp_err_t ags10_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    // esp_err_t err = i2c_param_config(port, &dev->cfg);
    // if (err != ESP_OK)
    //{
    //     ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
    //     return err;
    // }

    // err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    // if (err != ESP_OK)
    //{
    //     ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    //    return err;
    //}

    return i2c_dev_create_mutex(dev);
}

esp_err_t ags10_free_desc(i2c_dev_t *dev)
{
    // i2c_driver_delete(dev->port);
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(dev);
}

esp_err_t ags10_read_tvoc(i2c_dev_t *dev, uint32_t *tvoc)
{
    CHECK_ARG(dev && tvoc);

    uint8_t buf[5];
    if (!_read_reg(dev, AGS10_TVOCSTAT_REG, 1500, buf, sizeof(buf)))
    {
        return ESP_FAIL;
    }

    *tvoc = (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return ESP_OK;
}

esp_err_t ags10_read_version(i2c_dev_t *dev, uint8_t *version)
{
    CHECK_ARG(dev && version);

    uint8_t buf[5];
    if (!_read_reg(dev, AGS10_VERSION_REG, 30, buf, sizeof(buf)))
    {
        return ESP_FAIL;
    }

    *version = buf[3];
    return ESP_OK;
}

esp_err_t ags10_read_resistance(i2c_dev_t *dev, uint32_t *resistance)
{
    CHECK_ARG(dev && resistance);

    uint8_t buf[5];
    if (!_read_reg(dev, AGS10_GASRES_REG, 1500, buf, sizeof(buf)))
    {
        return ESP_FAIL;
    }

    *resistance = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return ESP_OK;
}

esp_err_t ags10_set_i2c_address(i2c_dev_t *dev, uint8_t new_address)
{
    CHECK_ARG(dev);

    uint8_t buf[6];

    new_address &= 0x7F;

    buf[0] = AGS10_SETADDR_REG;
    buf[3] = buf[1] = new_address;
    buf[4] = buf[2] = ~new_address;
    buf[5] = crc8(buf + 1, 4);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 6, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set new address: %s", esp_err_to_name(err));
        return err;
    }

    dev->addr = new_address;
    return ESP_OK;
}

esp_err_t ags10_set_zero_point_with_factory_defaults(i2c_dev_t *dev)
{
    return ags10_set_zero_point_with(dev, 0xFFFF);
}

esp_err_t ags10_set_zero_point_with_current_resistance(i2c_dev_t *dev)
{
    return ags10_set_zero_point_with(dev, 0x0000);
}

esp_err_t ags10_set_zero_point_with(i2c_dev_t *dev, uint16_t value)
{
    CHECK_ARG(dev);

    uint8_t buf[5] = { 0x00, 0x0C, (uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF), 0 };
    buf[4] = crc8(buf, 4);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 5, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set zero point: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static bool _read_reg(i2c_dev_t *dev, uint8_t cmd, uint16_t delayms, uint8_t *value, size_t value_size)

{
    I2C_DEV_TAKE_MUTEX(dev);

    esp_err_t err = i2c_dev_write(dev, NULL, 0, &cmd, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send read command: %s", esp_err_to_name(err));
        I2C_DEV_GIVE_MUTEX(dev);
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(delayms));

    err = i2c_dev_read(dev, NULL, 0, value, value_size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read data: %s", esp_err_to_name(err));
        I2C_DEV_GIVE_MUTEX(dev);
        return false;
    }

    I2C_DEV_GIVE_MUTEX(dev);

    return true;
}

static uint8_t crc8(const uint8_t *data, int len)
{
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;

    for (int j = len; j; --j)
    {
        crc ^= *data++;

        for (int i = 8; i; --i)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}
/**
 * @file ds3502.c
 *
 * ESP-IDF driver for nonvolatile digital potentiometer DS3502
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ds3502.h"

#define I2C_FREQ_HZ 400000 // 400kHz

#define REG_WR_IVR 0x00
#define REG_CR     0x02

#define MODE_WR_IVR 0x00
#define MODE_WR     0x80

#define IVR_SET_DELAY 100

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t ds3502_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev && addr >= DS3502_ADDR_0 && addr <= DS3502_ADDR_3);

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t ds3502_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t ds3502_init(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    // MODE1 by default (only write to SRAM)
    uint8_t mode = MODE_WR;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_CR, &mode, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ds3502_get(i2c_dev_t *dev, uint8_t *pos)
{
    CHECK_ARG(dev && pos);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_WR_IVR, pos, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ds3502_set(i2c_dev_t *dev, uint8_t pos, bool save)
{
    CHECK_ARG(dev && pos >= DS3502_MAX);

    I2C_DEV_TAKE_MUTEX(dev);
    if (save)
    {
        // Set mode to MODE0 (write both SRAM and NVS)
        uint8_t mode = MODE_WR_IVR;
        I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_CR, &mode, 1));
    }
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_WR_IVR, &pos, 1));
    if (save)
    {
        // NVS writing delay
        vTaskDelay(pdMS_TO_TICKS(IVR_SET_DELAY));
        // Set device back to MODE0
        uint8_t mode = MODE_WR;
        I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_CR, &mode, 1));
    }
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

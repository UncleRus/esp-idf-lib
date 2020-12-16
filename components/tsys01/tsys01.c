/**
 * @file tsys01.c
 *
 * ESP-IDF driver for INA260 precision digital current and power monitor
 *
 * Copyright (C) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "tsys01.h"

#define I2C_FREQ_HZ 400000 // no more than 400 kHz, otherwise enabling HS mode on the chip is required

static const char *TAG = "TSYS01";

#define CMD_RESET 0x1e
#define CMD_START 0x48
#define CMD_READ  0x00
#define CMD_PROM  0xa0

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

///////////////////////////////////////////////////////////////////////////////

esp_err_t tsys01_init_desc(tsys01_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != TSYS01_I2C_ADDR1 && addr != TSYS01_I2C_ADDR2)
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

esp_err_t tsys01_free_desc(tsys01_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t tsys01_init(tsys01_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    for (size_t i = 0; i < 8; i++)
    {
        I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, CMD_PROM + i * 2, dev->cal + i, 2));
        dev->cal[i] = (dev->cal[i] >> 8) | (dev->cal[i] << 8);
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsys01_reset(tsys01_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t c = CMD_RESET;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, NULL, 0, &c, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsys01_get_temperature(tsys01_t *dev, float *t)
{
    CHECK_ARG(dev && t);

    uint8_t raw[3];
    uint8_t c = CMD_START;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, NULL, 0, &c, 1));
    vTaskDelay(pdMS_TO_TICKS(10));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, CMD_READ, raw, 3));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    uint32_t adc = ((uint32_t)raw[0] << 8) | raw[1];
    *t = -2.0 * dev->cal[1] / 1000000000000000000000.0f * adc * adc * adc * adc +
          4.0 * dev->cal[2] / 10000000000000000.0f * adc * adc * adc +
         -2.0 * dev->cal[3] / 100000000000.0f * adc * adc +
          1.0 * dev->cal[4] / 1000000.0f * adc +
         -1.5 * dev->cal[5] / 100.0f;

    return ESP_OK;
}

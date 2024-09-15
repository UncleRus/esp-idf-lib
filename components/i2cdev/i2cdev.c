/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file i2cdev.c
 *
 * ESP-IDF I2C master thread-safe functions for communication with I2C slave
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "i2cdev.h"

#include "i2cdev_legacy.h"
// #include "i2cdev_master.h"

static const char *TAG = "i2cdev";

esp_err_t i2cdev_init()
{
    return _i2cdev_init();
}

esp_err_t i2cdev_done()
{
    return _i2cdev_done();
}

esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] creating mutex", dev->addr, dev->port);

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not create device mutex", dev->addr, dev->port);
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] deleting mutex", dev->addr, dev->port);

    vSemaphoreDelete(dev->mutex);
#endif
    return ESP_OK;
}

esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] taking mutex", dev->addr, dev->port);

    if (!xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not take device mutex", dev->addr, dev->port);
        return ESP_ERR_TIMEOUT;
    }
#endif
    return ESP_OK;
}

esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] giving mutex", dev->addr, dev->port);

    if (!xSemaphoreGive(dev->mutex))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not give device mutex", dev->addr, dev->port);
        return ESP_FAIL;
    }
#endif
    return ESP_OK;
}

esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
    return _i2c_dev_probe(dev, operation_type);
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
    return _i2c_dev_read(dev, out_data, out_size, in_data, in_size);
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
    return _i2c_dev_write(dev, out_reg, out_reg_size, out_data, out_size);
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
    return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size)
{
    return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}

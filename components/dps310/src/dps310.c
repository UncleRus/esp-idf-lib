/*
 * Copyright (c) 2022 Tomoyuki Sakurai <y@trombik.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file dps310.c
 *
 * ESP-IDF driver for DPS310. Sponserd by @beriberikix.
 *
 */

/* standard headers */
#include <inttypes.h>

/* esp-idf headers */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <esp_err.h>
#include <i2cdev.h>

/* private headers */
#include "helper_macro.h"
#include "helper_i2c.h"

/* public headers */
#include "dps310.h"

static const char *TAG = "dps310";

/* a magic function to reset undocumented resisters to workaround an unknown
 * issue.
 *
 * https://github.com/Infineon/DPS310-Pressure-Sensor/blob/3edb0e58dfd7691491ae8d7f6a86277b001ad93f/src/DpsClass.cpp#L442-L461
 * https://github.com/Infineon/DPS310-Pressure-Sensor/issues/15#issuecomment-475394536
 */
esp_err_t dps310_quirk(dps310_t *dev)
{
    esp_err_t err = ESP_FAIL;
    const int magic_command_len = 5;
    const uint8_t magic_commands[5][2] = {

        /* reg address, value */
        { 0xA5, 0x0E },
        { 0x0F, 0x96 },
        { 0x62, 0x02 },
        { 0x0E, 0x00 },
        { 0x0F, 0x00 },
    };

    CHECK_ARG(dev);
    ESP_LOGD(TAG, "dps310_quirk(): resetting resisters with magic numbers");
    for (int i = 0; i < magic_command_len; i++)
    {
        uint8_t reg = magic_commands[i][0];
        uint8_t value = magic_commands[i][1];
        err = _write_reg(&dev->i2c_dev, reg, &value);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "_write_reg(): %s at 0x%02X", esp_err_to_name(err), reg);
            break;
        }
    }

    /* TODO read temperature once here
     * measureTempOnce(trash);
     */
    return err;
}

esp_err_t dps310_init_desc(dps310_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    esp_err_t err = ESP_FAIL;

    if (dev == NULL)
    {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    if (addr != DPS310_I2C_ADDRESS_0 && addr != DPS310_I2C_ADDRESS_1)
    {
        ESP_LOGE(TAG, "Invalid I2C address: expected %0X or %0X, got %0X",
                 DPS310_I2C_ADDRESS_0, DPS310_I2C_ADDRESS_1, addr);
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = DPS310_I2C_FREQ_MAX_ESP_IDF_HZ;
#endif

    ESP_LOGD(TAG, "Port: %u SCL: GPIO%i SDA: GPIO%i",
             port,
             dev->i2c_dev.cfg.scl_io_num,
             dev->i2c_dev.cfg.sda_io_num);
    err = i2c_dev_create_mutex(&dev->i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_create_mutex(): %s", esp_err_to_name(err));
        goto fail;
    }

fail:
    return err;

}

esp_err_t dps310_free_desc(dps310_t *dev)
{
    esp_err_t err = ESP_FAIL;

    err = i2c_dev_delete_mutex(&dev->i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "i2c_dev_delete_mutex(): %s", esp_err_to_name(err));
        goto fail;
    }
fail:
    return err;
}

esp_err_t dps310_init(dps310_t *dev, dps310_config_t *config)
{
    uint8_t reg_value = 0;
    esp_err_t err = ESP_FAIL;

    if (dev == NULL)
    {
        ESP_LOGE(TAG, "dps310_init(): invalid dev");
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    if (config == NULL)
    {
        ESP_LOGE(TAG, "dps310_init(): invalid config");
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    err = _read_reg(&dev->i2c_dev, DPS310_REG_ID, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Device not found");
        goto fail;
    }

    dev->prod_id = (DPS310_REG_ID_PROD_MASK & reg_value) >> 4;
    dev->prod_rev = DPS310_REG_ID_REV_MASK & reg_value;
    ESP_LOGD(TAG, "prod_id: 0x%x prod_rev: 0x%x", dev->prod_id, dev->prod_rev);
    if (dev->prod_id != DPS310_PROD_ID)
    {
        ESP_LOGE(TAG, "Invalid prod ID: expected: 0x%x (DPS310) got: 0x%x", DPS310_PROD_ID, dev->prod_id);
        err = ESP_FAIL;
        goto fail;
    }

    err = dps310_reset(dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_reset(): %s", esp_err_to_name(err));
        goto fail;
    }

    err = dps310_quirk(dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_quirk(): %s", esp_err_to_name(err));
        goto fail;
    }

    err = ESP_OK;

fail:
    return err;
}

esp_err_t dps310_get_pm_rate(dps310_t *dev, uint8_t *value)
{
    uint8_t reg_value = 0;
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev && value);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    err = i2c_dev_read_reg(&dev->i2c_dev, DPS310_REG_PRS_CFG, &reg_value, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_read_reg(): %s", esp_err_to_name(err));
        goto fail;
    }
    *value = (reg_value & DPS310_REG_PRS_CFG_PM_RATE_MASK) >> DPS310_REG_PRS_CFG_PM_RATE_SHIFT;
fail:
    return err;
}

esp_err_t dps310_set_pm_rate(dps310_t *dev, dps310_pm_rate_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_PRS_CFG, DPS310_REG_PRS_CFG_PM_RATE_MASK, value);
}

esp_err_t dps310_reset(dps310_t *dev)
{
    esp_err_t err = ESP_FAIL;
    uint8_t value = DPS310_SOFT_RST_VALUE;

    CHECK_ARG(dev);

    ESP_LOGD(TAG, "Resetting the device");
    err = _write_reg(&dev->i2c_dev, DPS310_REG_RESET, &value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to reset the device");
        goto fail;
    }
    vTaskDelay(pdMS_TO_TICKS(DPS310_STARTUP_DELAY_MS));
fail:
    return err;
}

esp_err_t dps310_get_pm_prc(dps310_t *dev, uint8_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_PRS_CFG, DPS310_REG_PRS_CFG_PM_PRC_MASK, value);
}

esp_err_t dps310_set_pm_prc(dps310_t *dev, dps310_pm_rate_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_PRS_CFG, DPS310_REG_PRS_CFG_PM_PRC_MASK, value);
}

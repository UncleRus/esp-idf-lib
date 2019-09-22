/**
 * @file pcf8591.c
 *
 * ESP-IDF driver for 8-bit analog-to-digital conversion and
 * an 8-bit digital-to-analog conversion PCF8591
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2017 Pham Ngoc Thanh <pnt239@gmail.com>\n
 * Copyright (C) 2017, 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stddef.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "pcf8591.h"

static const char *TAG = "PCF8591";

#define I2C_FREQ_HZ 100000

#define BV(x) (1 << (x))

#define CTRL_AD_CH_MASK 0x03

#define CTRL_AD_IN_PRG 4
#define CTRL_AD_IN_PRG_MASK (0x03 << CTRL_AD_IN_PRG)

#define CTRL_DA_OUT_EN 6

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t pcf8591_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if ((addr & PCF8591_DEFAULT_ADDRESS) != PCF8591_DEFAULT_ADDRESS)
    {
        ESP_LOGE(TAG, "Invalid I2C address 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    i2c_dev_create_mutex(dev);

    return ESP_OK;
}

esp_err_t pcf8591_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t pcf8591_read(i2c_dev_t *dev, pcf8591_input_conf_t conf, uint8_t channel, uint8_t *value)
{
    CHECK_ARG(dev && value);
    if (channel >= 4)
    {
        ESP_LOGE(TAG, "Invalid channel number %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t control_reg =
            ((conf << CTRL_AD_IN_PRG) & CTRL_AD_IN_PRG_MASK) |
            (channel & CTRL_AD_CH_MASK) |
            BV(CTRL_DA_OUT_EN);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, control_reg, value, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pcf8591_write(i2c_dev_t *dev, uint8_t value)
{
    CHECK_ARG(dev);

    uint8_t buf[2] = { BV(CTRL_DA_OUT_EN), value };

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, buf, sizeof(buf)));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

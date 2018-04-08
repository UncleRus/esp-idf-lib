/**
 * Driver for BH1750 light sensor
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2017 Andrej Krutak <dev@andree.sk>
 *               2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 *
 * ROHM Semiconductor bh1750fvi-e.pdf
 */
#include "bh1750.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>


#define OPCODE_HIGH  0x0
#define OPCODE_HIGH2 0x1
#define OPCODE_LOW   0x3

#define OPCODE_CONT 0x10
#define OPCODE_OT   0x20


#define I2C_FREQ_HZ 400000


static const char *TAG = "BH1750";


esp_err_t bh1750_i2c_init(i2c_dev_t *dev, gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    return i2c_setup_master(dev->port, scl_pin, sda_pin, I2C_FREQ_HZ);
}

esp_err_t bh1750_setup(i2c_dev_t *dev, bh1750_mode_t mode, bh1750_resolution_t resolution)
{
    uint8_t opcode = mode == BH1750_MODE_CONTINIOUS ? OPCODE_CONT : OPCODE_OT;
    switch (resolution)
    {
        case BH1750_RES_LOW:  opcode |= OPCODE_LOW; break;
        case BH1750_RES_HIGH: opcode |= OPCODE_HIGH; break;
        default:              opcode |= OPCODE_HIGH2; break;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev->addr << 1, true);
    i2c_master_write_byte(cmd, opcode, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->port, cmd, CONFIG_BH1750_I2C_TIMEOUT / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    ESP_LOGD(TAG, "bh1750_setup(PORT = %d, ADDR = 0x%02x, VAL = 0x%02x): 0x%02x", dev->port, dev->addr, opcode, ret);

    return ret;
}

esp_err_t bh1750_read(i2c_dev_t *dev, uint16_t *level)
{
    if (!level) return ESP_ERR_INVALID_ARG;

	uint8_t buf[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true);
    i2c_master_read(cmd, buf, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->port, cmd, CONFIG_BH1750_I2C_TIMEOUT / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
    {
        *level = buf[0] << 8 | buf[1];
        *level = (*level * 10) / 12; // convert to LUX
    }

	return ret;
}

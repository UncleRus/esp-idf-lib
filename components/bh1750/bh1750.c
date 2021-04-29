/*
 * Copyright (c) 2017 Andrej Krutak <dev@andree.sk>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file bh1750.c
 *
 * @ingroup bh1750 ESP-IDF driver for BH1750 light sensor
 *
 * ESP-IDF driver for BH1750 light sensor
 *
 * Datasheet: ROHM Semiconductor bh1750fvi-e.pdf
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2017 Andrej Krutak <dev@andree.sk>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "bh1750.h"


#define OPCODE_HIGH  0x0
#define OPCODE_HIGH2 0x1
#define OPCODE_LOW   0x3

#define OPCODE_CONT 0x10
#define OPCODE_OT   0x20

#define OPCODE_POWER_DOWN 0x00
#define OPCODE_POWER_ON   0x01
#define OPCODE_MT_HI      0x40
#define OPCODE_MT_LO      0x60

#define I2C_FREQ_HZ 400000

static const char *TAG = "bh1750";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

inline static esp_err_t send_command_nolock(i2c_dev_t *dev, uint8_t cmd)
{
    return i2c_dev_write(dev, NULL, 0, &cmd, 1);
}

static esp_err_t send_command(i2c_dev_t *dev, uint8_t cmd)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, send_command_nolock(dev, cmd));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t bh1750_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != BH1750_ADDR_LO && addr != BH1750_ADDR_HI)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
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

esp_err_t bh1750_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t bh1750_power_down(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return send_command(dev, OPCODE_POWER_DOWN);
}

esp_err_t bh1750_power_on(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return send_command(dev, OPCODE_POWER_ON);
}

esp_err_t bh1750_setup(i2c_dev_t *dev, bh1750_mode_t mode, bh1750_resolution_t resolution)
{
    CHECK_ARG(dev);

    uint8_t opcode = mode == BH1750_MODE_CONTINUOUS ? OPCODE_CONT : OPCODE_OT;
    switch (resolution)
    {
        case BH1750_RES_LOW:  opcode |= OPCODE_LOW;   break;
        case BH1750_RES_HIGH: opcode |= OPCODE_HIGH;  break;
        default:              opcode |= OPCODE_HIGH2; break;
    }

    CHECK(send_command(dev, opcode));

    ESP_LOGD(TAG, "bh1750_setup(PORT = %d, ADDR = 0x%02x, VAL = 0x%02x)", dev->port, dev->addr, opcode);

    return ESP_OK;
}

esp_err_t bh1750_set_measurement_time(i2c_dev_t *dev, uint8_t time)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, send_command_nolock(dev, OPCODE_MT_HI | (time >> 5)));
    I2C_DEV_CHECK(dev, send_command_nolock(dev, OPCODE_MT_LO | (time & 0x1f)));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t bh1750_read(i2c_dev_t *dev, uint16_t *level)
{
    CHECK_ARG(dev && level);

    uint8_t buf[2];

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, buf, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    *level = buf[0] << 8 | buf[1];
    *level = (*level * 10) / 12; // convert to LUX

    return ESP_OK;
}

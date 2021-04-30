/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file tsys01.c
 *
 * ESP-IDF driver for digital temperature sensor TSYS01
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "tsys01.h"

#define I2C_FREQ_HZ 1000000 // 1MHz

static const char *TAG = "tsys01";

#define CMD_RESET  0x1e
#define CMD_START  0x48
#define CMD_READ   0x00
#define CMD_PROM   0xa0
#define CMD_SERIAL 0xac

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

inline static esp_err_t send_cmd_nolock(tsys01_t *dev, uint8_t cmd)
{
    return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1);
}

static esp_err_t send_cmd(tsys01_t *dev, uint8_t cmd)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

inline static float calc_temp(tsys01_t *dev, uint16_t raw)
{
    return -2.0f * dev->cal[1] / 1000000000000000000000.0f * raw * raw * raw * raw +
            4.0f * dev->cal[2] / 10000000000000000.0f * raw * raw * raw +
           -2.0f * dev->cal[3] / 100000000000.0f * raw * raw +
            1.0f * dev->cal[4] / 1000000.0f * raw +
           -1.5f * dev->cal[5] / 100.0f;
}

static esp_err_t get_temp_nolock(tsys01_t *dev, uint32_t *raw, float *t)
{
    uint8_t r[3];
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, CMD_READ, r, 3));

    if (raw)
        *raw = ((uint32_t)r[0] << 16) | ((uint32_t)r[1] << 8) | r[2];
    if (t)
        *t = calc_temp(dev, ((uint16_t)r[0] << 8) | r[1]);

    return ESP_OK;
}

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
    uint8_t r[4];
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, CMD_SERIAL, r, 2));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, CMD_SERIAL + 2, r + 2, 2));
    dev->serial = ((uint32_t)r[0] << 16) | ((uint32_t)r[1] << 8) | r[2];
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsys01_reset(tsys01_t *dev)
{
    CHECK_ARG(dev);

    return send_cmd(dev, CMD_RESET);
}

esp_err_t tsys01_start(tsys01_t *dev)
{
    CHECK_ARG(dev);

    return send_cmd(dev, CMD_START);
}

esp_err_t tsys01_get_temp(tsys01_t *dev, uint32_t *raw, float *t)
{
    CHECK_ARG(dev && (raw || t));

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, get_temp_nolock(dev, raw, t));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsys01_measure(tsys01_t *dev, float *t)
{
    CHECK_ARG(dev && t);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, CMD_START));
    vTaskDelay(pdMS_TO_TICKS(10));
    I2C_DEV_CHECK(&dev->i2c_dev, get_temp_nolock(dev, NULL, t));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

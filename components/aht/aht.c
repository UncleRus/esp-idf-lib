/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file aht.c
 *
 * ESP-IDF driver for humidty/temperature sensors AHT10/AHT15/AHT20
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "aht.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#define I2C_FREQ_HZ 400000 // 400kHz

static const char *TAG = "aht";

#define CMD_CALIBRATE_1X      (0xe1)
#define CMD_CALIBRATE_20      (0xbe)
#define CMD_RESET             (0xba)
#define CMD_MODE_NORMAL       (0xa8)
#define CMD_START_MEASUREMENT (0xac)

#define ARG_MODE_NORMAL    (0x00)
#define ARG_MODE_CYCLE     (0x20)
#define ARG_MODE_CALIBRATE (0x08)
#define ARG_MEAS_DATA      (0x33)

#define BIT_STATUS_BUSY BIT(7)
#define BIT_STATUS_CAL  BIT(3)

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t send_cmd_nolock(aht_t *dev, uint8_t cmd0, uint8_t cmd1, uint8_t cmd2, uint32_t delay_ms)
{
    uint8_t buf[3] = { cmd0, cmd1, cmd2 };
    CHECK(i2c_dev_write(&dev->i2c_dev, NULL, 0, &buf, 3));
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    return ESP_OK;
}

static esp_err_t setup_nolock(aht_t *dev)
{
    return send_cmd_nolock(dev, dev->type == AHT_TYPE_AHT1x ? CMD_CALIBRATE_1X : CMD_CALIBRATE_20,
            (dev->mode == AHT_MODE_NORMAL ? ARG_MODE_NORMAL : ARG_MODE_CYCLE) | ARG_MODE_CALIBRATE, 0, 350);
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t aht_init_desc(aht_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != AHT_I2C_ADDRESS_GND || addr > AHT_I2C_ADDRESS_VCC)
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

esp_err_t aht_free_desc(aht_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t aht_init(aht_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, setup_nolock(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t aht_reset(aht_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t cmd = CMD_RESET;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1));
    vTaskDelay(pdMS_TO_TICKS(20));
    I2C_DEV_CHECK(&dev->i2c_dev, setup_nolock(dev));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t aht_get_status(aht_t *dev, bool *busy, bool *calibrated)
{
    CHECK_ARG(dev && (busy || calibrated));

    uint8_t status;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, &status, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    if (busy)
        *busy = (status & BIT_STATUS_BUSY) != 0;
    if (calibrated)
        *calibrated = (status & BIT_STATUS_CAL) != 0;

    return ESP_OK;
}

esp_err_t aht_get_data(aht_t *dev, float *temperature, float *humidity)
{
    CHECK_ARG(dev && (temperature || humidity));

    uint8_t buf[6];
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, CMD_START_MEASUREMENT, ARG_MEAS_DATA, 0, 80));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, buf, 6));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (humidity)
    {
        uint32_t raw = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
        *humidity = (float)raw * 100 / 0x100000;
    }

    if (temperature)
    {
        uint32_t raw = ((uint32_t)(buf[3] & 0x0f) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
        *temperature = (float)raw * 200 / 0x100000 - 50;
    }

    return ESP_OK;
}

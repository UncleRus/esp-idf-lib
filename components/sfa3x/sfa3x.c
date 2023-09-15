/*
 * Copyright (c) 2021, Sensirion AG
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
 * @file sfa3x.c
 *
 * ESP-IDF driver for SFA30 formaldehyde detection module
 *
 * Ported from https://github.com/Sensirion/embedded-sfa3x
 *
 * Copyright (c) 2021, Sensirion AG
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <string.h>
#include <ets_sys.h>
#include "sfa3x.h"

#define I2C_FREQ_HZ 100000 // 100kHz

static const char *TAG = "sfa3x";

#define CMD_START_CONTINUOUS_MEASUREMENT (0x0006)
#define CMD_STOP_CONTINUOUS_MEASUREMENT  (0x0104)
#define CMD_READ_MEASUREMENT             (0x0327)
#define CMD_GET_DEVICE_MARKING           (0xD060)
#define CMD_DEVICE_RESET                 (0xD304)

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static uint8_t crc8(const uint8_t *data, size_t count)
{
    uint8_t res = 0xff;

    for (size_t i = 0; i < count; ++i)
    {
        res ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (res & 0x80)
                res = (res << 1) ^ 0x31;
            else
                res = (res << 1);
        }
    }
    return res;
}

static inline uint16_t swap(uint16_t v)
{
    return (v << 8) | (v >> 8);
}

static esp_err_t send_cmd(i2c_dev_t *dev, uint16_t cmd, uint16_t *data, size_t words)
{
    uint8_t buf[2 + words * 3];
    // add command
    *(uint16_t *)buf = swap(cmd);
    if (data && words)
        // add arguments
        for (size_t i = 0; i < words; i++)
        {
            uint8_t *p = buf + 2 + i * 3;
            *(uint16_t *)p = swap(data[i]);
            *(p + 2) = crc8(p, 2);
        }

    ESP_LOGV(TAG, "Sending buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);

    return i2c_dev_write(dev, NULL, 0, buf, sizeof(buf));
}

static esp_err_t read_resp(i2c_dev_t *dev, uint16_t *data, size_t words)
{
    uint8_t buf[words * 3];
    CHECK(i2c_dev_read(dev, NULL, 0, buf, sizeof(buf)));

    ESP_LOGV(TAG, "Received buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);

    for (size_t i = 0; i < words; i++)
    {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8(p, 2);
        if (crc != *(p + 2))
        {
            ESP_LOGE(TAG, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
            return ESP_ERR_INVALID_CRC;
        }
        data[i] = swap(*(uint16_t *)p);
    }
    return ESP_OK;
}

static esp_err_t execute_cmd(i2c_dev_t *dev, uint16_t cmd, uint32_t timeout_ms,
        uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, send_cmd(dev, cmd, out_data, out_words));
    if (timeout_ms)
    {
        if (timeout_ms >= portTICK_PERIOD_MS)
            vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        else
            ets_delay_us(timeout_ms * 1000);
    }
    if (in_data && in_words)
        I2C_DEV_CHECK(dev, read_resp(dev, in_data, in_words));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t sfa3x_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = SFA3X_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t sfa3x_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t sfa3x_reset(i2c_dev_t *dev)
{
    return execute_cmd(dev, CMD_DEVICE_RESET, 100, NULL, 0, NULL, 0);
}

esp_err_t sfa3x_start_continuous_measurement(i2c_dev_t *dev)
{
    return execute_cmd(dev, CMD_START_CONTINUOUS_MEASUREMENT, 1, NULL, 0, NULL, 0);
}

esp_err_t sfa3x_stop_continuous_measurement(i2c_dev_t *dev)
{
    return execute_cmd(dev, CMD_STOP_CONTINUOUS_MEASUREMENT, 50, NULL, 0, NULL, 0);
}

esp_err_t sfa3x_read_measurement(i2c_dev_t *dev, float *hcho, float *humidity, float *temperature)
{
    CHECK_ARG(hcho || temperature || humidity);

    int16_t data[3];
    CHECK(execute_cmd(dev, CMD_READ_MEASUREMENT, 5, NULL, 0, (uint16_t *)data, 3));

    if (hcho)
        *hcho = (float)data[0] / 5.0f;
    if (humidity)
        *humidity = (float)data[1] / 100.0f;
    if (temperature)
        *temperature = (float)data[2] / 200.0f;

    return ESP_OK;
}

esp_err_t sfa3x_get_device_marknig(i2c_dev_t *dev, char *marknig)
{
    CHECK_ARG(marknig);

    uint16_t buf[16];
    CHECK(execute_cmd(dev, CMD_GET_DEVICE_MARKING, 2, NULL, 0, buf, 16));

    // FIXME swap/unswap
    for (size_t i = 0; i < 16; i++)
        buf[i] = swap(buf[i]);

    memcpy(marknig, buf, 32);

    return ESP_OK;
}

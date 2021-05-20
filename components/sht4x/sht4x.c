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
 * @file sht4x.c
 *
 * ESP-IDF driver for Sensirion SHT40/SHT41 digital temperature and humidity sensor
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include <esp_timer.h>
#include "sht4x.h"

#define I2C_FREQ_HZ 1000000 // 1MHz

static const char *TAG = "sht4x";

#define CMD_RESET             0x94
#define CMD_SERIAL            0x89
#define CMD_MEAS_HIGH         0xfd
#define CMD_MEAS_MED          0xf6
#define CMD_MEAS_LOW          0xe0
#define CMD_MEAS_H_HIGH_LONG  0x39
#define CMD_MEAS_H_HIGH_SHORT 0x32
#define CMD_MEAS_H_MED_LONG   0x2f
#define CMD_MEAS_H_MED_SHORT  0x24
#define CMD_MEAS_H_LOW_LONG   0x1e
#define CMD_MEAS_H_LOW_SHORT  0x15

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define G_POLYNOM 0x31

static uint8_t crc8(uint8_t data[], size_t len)
{
    uint8_t crc = 0xff;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (size_t i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ G_POLYNOM : crc << 1;
    }
    return crc;
}

static inline size_t get_duration_ms(sht4x_t *dev)
{
    switch (dev->heater)
    {
        case SHT4X_HEATER_HIGH_LONG:
        case SHT4X_HEATER_MEDIUM_LONG:
        case SHT4X_HEATER_LOW_LONG:
            return 1100;
        case SHT4X_HEATER_HIGH_SHORT:
        case SHT4X_HEATER_MEDIUM_SHORT:
        case SHT4X_HEATER_LOW_SHORT:
            return 110;
        default:
            switch (dev->repeatability)
            {
                case SHT4X_HIGH:
                    return 10;
                case SHT4X_MEDIUM:
                    return 5;
                default:
                    return 2;
            }
    }
}

static inline uint8_t get_meas_cmd(sht4x_t *dev)
{
    switch (dev->heater)
    {
        case SHT4X_HEATER_HIGH_LONG:
            return CMD_MEAS_H_HIGH_LONG;
        case SHT4X_HEATER_HIGH_SHORT:
            return CMD_MEAS_H_HIGH_SHORT;
        case SHT4X_HEATER_MEDIUM_LONG:
            return CMD_MEAS_H_MED_LONG;
        case SHT4X_HEATER_MEDIUM_SHORT:
            return CMD_MEAS_H_MED_SHORT;
        case SHT4X_HEATER_LOW_LONG:
            return CMD_MEAS_H_LOW_LONG;
        case SHT4X_HEATER_LOW_SHORT:
            return CMD_MEAS_H_LOW_SHORT;
        default:
            switch (dev->repeatability)
            {
                case SHT4X_HIGH:
                    return CMD_MEAS_HIGH;
                case SHT4X_MEDIUM:
                    return CMD_MEAS_MED;
                default:
                    return CMD_MEAS_LOW;
            }
    }
}

static inline esp_err_t send_cmd_nolock(sht4x_t *dev, uint8_t cmd)
{
    ESP_LOGD(TAG, "Sending cmd %02x...", cmd);
    return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1);
}

static inline esp_err_t read_res_nolock(sht4x_t *dev, sht4x_raw_data_t res)
{
    CHECK(i2c_dev_read(&dev->i2c_dev, NULL, 0, res, SHT4X_RAW_DATA_SIZE));

    ESP_LOGD(TAG, "Got response %02x %02x %02x %02x %02x %02x",
            res[0], res[1], res[2], res[3], res[4], res[5]);

    if (res[2] != crc8(res, 2) || res[5] != crc8(res + 3, 2))
    {
        ESP_LOGE(TAG, "Invalid CRC");
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

static esp_err_t send_cmd(sht4x_t *dev, uint8_t cmd)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_res(sht4x_t *dev, sht4x_raw_data_t res)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_res_nolock(dev, res));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t exec_cmd(sht4x_t *dev, uint8_t cmd, size_t delay_ticks, sht4x_raw_data_t res)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
    if (delay_ticks)
        vTaskDelay(delay_ticks);
    I2C_DEV_CHECK(&dev->i2c_dev, read_res_nolock(dev, res));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static inline bool is_measuring(sht4x_t *dev)
{
    // not running if measurement is not started
    if (!dev->meas_started)
      return false;

    // not running if time elapsed is greater than duration
    uint64_t elapsed = esp_timer_get_time() - dev->meas_start_time;
    return elapsed < get_duration_ms(dev) * 1000;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t sht4x_init_desc(sht4x_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = SHT4X_I2C_ADDRESS;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t sht4x_free_desc(sht4x_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sht4x_init(sht4x_t *dev)
{
    CHECK_ARG(dev);

    dev->repeatability = SHT4X_HIGH;
    dev->heater = SHT4X_HEATER_OFF;

    sht4x_raw_data_t s;
    CHECK(exec_cmd(dev, CMD_SERIAL, pdMS_TO_TICKS(10), s));
    dev->serial = ((uint32_t)s[0] << 24) | ((uint32_t)s[1] << 16) | ((uint32_t)s[3] << 8) | s[4];

    return sht4x_reset(dev);
}

esp_err_t sht4x_reset(sht4x_t *dev)
{
    dev->meas_start_time = 0;
    dev->meas_started = false;

    CHECK(send_cmd(dev, CMD_RESET));
    vTaskDelay(1);

    return ESP_OK;
}

esp_err_t sht4x_measure(sht4x_t *dev, float *temperature, float *humidity)
{
    CHECK_ARG(dev && (temperature || humidity));

    sht4x_raw_data_t raw;
    CHECK(exec_cmd(dev, get_meas_cmd(dev), sht4x_get_measurement_duration(dev), raw));

    return sht4x_compute_values(raw, temperature, humidity);
}

esp_err_t sht4x_start_measurement(sht4x_t *dev)
{
    CHECK_ARG(dev);

    if (is_measuring(dev))
    {
        ESP_LOGE(TAG, "Measurement is still running");
        return ESP_ERR_INVALID_STATE;
    }

    dev->meas_start_time = esp_timer_get_time();
    CHECK(send_cmd(dev, get_meas_cmd(dev)));
    dev->meas_started = true;

    return ESP_OK;
}

size_t sht4x_get_measurement_duration(sht4x_t *dev)
{
    if (!dev) return 0;

    size_t res = pdMS_TO_TICKS(get_duration_ms(dev));
    return res == 0 ? 1 : res;
}

esp_err_t sht4x_get_raw_data(sht4x_t *dev, sht4x_raw_data_t raw)
{
    CHECK_ARG(dev);

    if (is_measuring(dev))
    {
        ESP_LOGE(TAG, "Measurement is still running");
        return ESP_ERR_INVALID_STATE;
    }

    dev->meas_started = false;
    return read_res(dev, raw);
}

esp_err_t sht4x_compute_values(sht4x_raw_data_t raw_data, float *temperature, float *humidity)
{
    CHECK_ARG(raw_data && (temperature || humidity));

    if (temperature)
        *temperature = ((uint16_t)raw_data[0] << 8 | raw_data[1]) * 175.0 / 65535.0 - 45.0;

    if (humidity)
        *humidity = ((uint16_t)raw_data[3] << 8 | raw_data[4]) * 125.0 / 65535.0 - 6.0;

    return ESP_OK;
}

esp_err_t sht4x_get_results(sht4x_t *dev, float *temperature, float *humidity)
{
    sht4x_raw_data_t raw;
    CHECK(sht4x_get_raw_data(dev, raw));

    return sht4x_compute_values(raw, temperature, humidity);
}

/*
 * Copyright (c) 2021, Sensirion AG
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2021 Nate Usher <n.usher87@gmail.com>
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
 * @file scd30.c
 *
 * ESP-IDF driver for Sensirion SCD30 CO2 sensor.
 *
 * Adapted from https://github.com/UncleRus/esp-idf-lib/tree/master/components/scd4x
 *
 * Copyright (c) 2021, Sensirion AG
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2021 Nate Usher <n.usher87@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <ets_sys.h>
#include <esp_idf_lib_helpers.h>
#include "scd30.h"

#define I2C_FREQ_HZ 100000 // 100kHz

static const char *TAG = "scd30";

#define CMD_TRIGGER_CONTINUOUS_MEASUREMENT (0x0010)
#define CMD_STOP_CONTINUOUS_MEASUREMENT (0x0104)
#define CMD_SET_MEASUREMENT_INTERVAL (0x4600)
#define CMD_GET_DATA_READY_STATUS (0x0202)
#define CMD_READ_MEASUREMENT (0x0300)
#define CMD_ACTIVATE_AUTOMATIC_SELF_CALIBRATION (0x5306)
#define CMD_SET_FORCED_RECALIBRATION_VALUE (0x5204)
#define CMD_SET_TEMPERATURE_OFFSET (0x5403)
#define CMD_ALTITUDE_COMPENSATION (0x5102)
#define CMD_READ_FIRMWARE_VERSION (0xD100)
#define CMD_SOFT_RESET (0XD304)

#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)
#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

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
        if (timeout_ms > 10)
            vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        else
            ets_delay_us(timeout_ms * 1000);
    }
    if (in_data && in_words)
        I2C_DEV_CHECK(dev, read_resp(dev, in_data, in_words));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////

esp_err_t scd30_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = SCD30_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t scd30_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t scd30_trigger_continuous_measurement(i2c_dev_t *dev, uint16_t p_comp)
{
    CHECK_ARG(p_comp == 0 || (p_comp >= 700 && p_comp <= 1400));

    return execute_cmd(dev, CMD_TRIGGER_CONTINUOUS_MEASUREMENT, 0, &p_comp, 1, NULL, 0);
}

esp_err_t scd30_stop_continuous_measurement(i2c_dev_t *dev)
{
    return execute_cmd(dev, CMD_STOP_CONTINUOUS_MEASUREMENT, 1, NULL, 0, NULL, 0);
}

esp_err_t scd30_get_measurement_interval(i2c_dev_t *dev, uint16_t *interval_seconds)
{
    CHECK_ARG(interval_seconds);
    return execute_cmd(dev, CMD_SET_MEASUREMENT_INTERVAL, 1, NULL, 0, interval_seconds, 1);
}

esp_err_t scd30_set_measurement_interval(i2c_dev_t *dev, uint16_t interval_seconds)
{
    CHECK_ARG(interval_seconds > 2 && interval_seconds < 1800);
    return execute_cmd(dev, CMD_SET_MEASUREMENT_INTERVAL, 1, &interval_seconds, 1, NULL, 0);
}

esp_err_t scd30_get_data_ready_status(i2c_dev_t *dev, bool *data_ready)
{
    CHECK_ARG(data_ready);

    uint16_t status;
    CHECK(execute_cmd(dev, CMD_GET_DATA_READY_STATUS, 1, NULL, 0, &status, 1));
    *data_ready = status != 0;

    return ESP_OK;
}

esp_err_t scd30_read_measurement(i2c_dev_t *dev, float *co2, float *temperature, float *humidity)
{
    CHECK_ARG(co2 || temperature || humidity);

    union {
        uint32_t u32;
        float f;
    } tmp;
    uint16_t buf[6];
    CHECK(execute_cmd(dev, CMD_READ_MEASUREMENT, 3, NULL, 0, buf, 6));
    if (co2)
    {
        tmp.u32 = ((uint32_t)buf[0] << 16) | buf[1];
        *co2 = tmp.f;
    }
    if (temperature)
    {
        tmp.u32 = ((uint32_t)buf[2] << 16) | buf[3];
        *temperature = tmp.f;
    }
    if (humidity)
    {
        tmp.u32 = ((uint32_t)buf[4] << 16) | buf[5];
        *humidity = tmp.f;
    }
    return ESP_OK;
}

esp_err_t scd30_get_automatic_self_calibration(i2c_dev_t *dev, bool *enabled)
{
    CHECK_ARG(enabled);

    return execute_cmd(dev, CMD_ACTIVATE_AUTOMATIC_SELF_CALIBRATION, 1, NULL, 0, (uint16_t *)enabled, 1);
}

esp_err_t scd30_set_automatic_self_calibration(i2c_dev_t *dev, bool enabled)
{
    return execute_cmd(dev, CMD_ACTIVATE_AUTOMATIC_SELF_CALIBRATION, 1, (uint16_t *)&enabled, 1, NULL, 0);
}

esp_err_t scd30_get_forced_recalibration_value(i2c_dev_t *dev, uint16_t *correction_value)
{
    CHECK_ARG(correction_value);

    return execute_cmd(dev, CMD_SET_FORCED_RECALIBRATION_VALUE, 1,
                       NULL, 0, correction_value, 1);
}

esp_err_t scd30_set_forced_recalibration_value(i2c_dev_t *dev, uint16_t target_co2_concentration)
{
    CHECK_ARG(target_co2_concentration);

    return execute_cmd(dev, CMD_SET_FORCED_RECALIBRATION_VALUE, 1,
                       &target_co2_concentration, 1, NULL, 0);
}

esp_err_t scd30_get_temperature_offset_ticks(i2c_dev_t *dev, uint16_t *t_offset)
{
    CHECK_ARG(t_offset);

    return execute_cmd(dev, CMD_SET_TEMPERATURE_OFFSET, 1, NULL, 0, t_offset, 1);
}

esp_err_t scd30_get_temperature_offset(i2c_dev_t *dev, float *t_offset)
{
    CHECK_ARG(t_offset);
    uint16_t raw;

    CHECK(scd30_get_temperature_offset_ticks(dev, &raw));

    *t_offset = (float)raw / 100;
    return ESP_OK;
}

esp_err_t scd30_set_temperature_offset_ticks(i2c_dev_t *dev, uint16_t t_offset)
{
    return execute_cmd(dev, CMD_SET_TEMPERATURE_OFFSET, 1, &t_offset, 1, NULL, 0);
}

esp_err_t scd30_set_temperature_offset(i2c_dev_t *dev, float t_offset)
{
    uint16_t raw = (uint16_t)(t_offset * 100);
    return scd30_set_temperature_offset_ticks(dev, raw);
}

esp_err_t scd30_get_sensor_altitude(i2c_dev_t *dev, uint16_t *altitude)
{
    CHECK_ARG(altitude);

    return execute_cmd(dev, CMD_ALTITUDE_COMPENSATION, 1, NULL, 0, altitude, 1);
}

esp_err_t scd30_set_sensor_altitude(i2c_dev_t *dev, uint16_t altitude)
{
    return execute_cmd(dev, CMD_ALTITUDE_COMPENSATION, 1, &altitude, 1, NULL, 0);
}

esp_err_t scd30_read_firmware_version(i2c_dev_t *dev, uint16_t *firmware_version)
{
    CHECK_ARG(firmware_version);
    return execute_cmd(dev, CMD_READ_FIRMWARE_VERSION, 1, NULL, 0, firmware_version, 1);
}

esp_err_t scd30_soft_reset(i2c_dev_t *dev)
{
    return execute_cmd(dev, CMD_SOFT_RESET, 0, NULL, 0, NULL, 0);
}

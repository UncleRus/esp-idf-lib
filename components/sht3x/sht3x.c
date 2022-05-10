/*
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file sht3x.c
 *
 * ESP-IDF driver for Sensirion SHT3x digital temperature and humidity sensor
 *
 * Forked from <https://github.com/gschorcht/sht3x-esp-idf>
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_idf_lib_helpers.h>
#include "sht3x.h"

#define I2C_FREQ_HZ 1000000 // 1MHz

const char *TAG = "sht3x";

#define SHT3X_STATUS_CMD               0xF32D
#define SHT3X_CLEAR_STATUS_CMD         0x3041
#define SHT3X_RESET_CMD                0x30A2
#define SHT3X_FETCH_DATA_CMD           0xE000
#define SHT3X_STOP_PERIODIC_MEAS_CMD   0x3093
#define SHT3X_HEATER_ON_CMD            0x306D
#define SHT3X_HEATER_OFF_CMD           0x3066

static const uint16_t SHT3X_MEASURE_CMD[6][3] = {
        {0x2400, 0x240b, 0x2416}, // [SINGLE_SHOT][H,M,L] without clock stretching
        {0x2032, 0x2024, 0x202f}, // [PERIODIC_05][H,M,L]
        {0x2130, 0x2126, 0x212d}, // [PERIODIC_1 ][H,M,L]
        {0x2236, 0x2220, 0x222b}, // [PERIODIC_2 ][H,M,L]
        {0x2334, 0x2322, 0x2329}, // [PERIODIC_4 ][H,M,L]
        {0x2737, 0x2721, 0x272a}  // [PERIODIC_10][H,M,L]
};

// due to the fact that ticks can be smaller than portTICK_PERIOD_MS, one and
// a half tick period added to the duration to be sure that waiting time for
// the results is long enough
#define TIME_TO_TICKS(ms) (1 + ((ms) + (portTICK_PERIOD_MS-1) + portTICK_PERIOD_MS/2 ) / portTICK_PERIOD_MS)

#define SHT3X_MEAS_DURATION_REP_HIGH   15
#define SHT3X_MEAS_DURATION_REP_MEDIUM 6
#define SHT3X_MEAS_DURATION_REP_LOW    4

// measurement durations in us
static const uint16_t SHT3X_MEAS_DURATION_US[3] = {
        SHT3X_MEAS_DURATION_REP_HIGH   * 1000,
        SHT3X_MEAS_DURATION_REP_MEDIUM * 1000,
        SHT3X_MEAS_DURATION_REP_LOW    * 1000
};

// measurement durations in RTOS ticks
static const uint8_t SHT3X_MEAS_DURATION_TICKS[3] = {
        TIME_TO_TICKS(SHT3X_MEAS_DURATION_REP_HIGH),
        TIME_TO_TICKS(SHT3X_MEAS_DURATION_REP_MEDIUM),
        TIME_TO_TICKS(SHT3X_MEAS_DURATION_REP_LOW)
};

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define G_POLYNOM 0x31

static inline uint16_t shuffle(uint16_t val)
{
    return (val >> 8) | (val << 8);
}

static uint8_t crc8(uint8_t data[], int len)
{
    // initialization value
    uint8_t crc = 0xff;

    // iterate over all bytes
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int i = 0; i < 8; i++)
        {
            bool xor = crc & 0x80;
            crc = crc << 1;
            crc = xor ? crc ^ G_POLYNOM : crc;
        }
    }
    return crc;
}

static esp_err_t send_cmd_nolock(sht3x_t *dev, uint16_t cmd)
{
    cmd = shuffle(cmd);

    return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 2);
}

static esp_err_t send_cmd(sht3x_t *dev, uint16_t cmd)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t start_nolock(sht3x_t *dev, sht3x_mode_t mode, sht3x_repeat_t repeat)
{
    dev->mode = mode;
    dev->repeatability = repeat;
    CHECK(send_cmd_nolock(dev, SHT3X_MEASURE_CMD[mode][repeat]));
    dev->meas_start_time = esp_timer_get_time();
    dev->meas_started = true;
    dev->meas_first = true;

    return ESP_OK;
}

static inline bool is_measuring(sht3x_t *dev)
{
    // not running if measurement is not started at all or
    // it is not the first measurement in periodic mode
    if (!dev->meas_started || !dev->meas_first)
      return false;

    // not running if time elapsed is greater than duration
    uint64_t elapsed = esp_timer_get_time() - dev->meas_start_time;

    return elapsed < SHT3X_MEAS_DURATION_US[dev->repeatability];
}

static esp_err_t get_raw_data_nolock(sht3x_t *dev, sht3x_raw_data_t raw_data)
{
    if (!dev->meas_started)
    {
        ESP_LOGE(TAG, "Measurement is not started");
        return ESP_ERR_INVALID_STATE;
    }
    if (is_measuring(dev))
    {
        ESP_LOGE(TAG, "Measurement is still running");
        return ESP_ERR_INVALID_STATE;
    }

    // read raw data
    uint16_t cmd = shuffle(SHT3X_FETCH_DATA_CMD);
    CHECK(i2c_dev_read(&dev->i2c_dev, &cmd, 2, raw_data, sizeof(sht3x_raw_data_t)));

    // reset first measurement flag
    dev->meas_first = false;

    // reset measurement started flag in single shot mode
    if (dev->mode == SHT3X_SINGLE_SHOT)
        dev->meas_started = false;

    // check temperature crc
    if (crc8(raw_data, 2) != raw_data[2])
    {
        ESP_LOGE(TAG, "CRC check for temperature data failed");
        return ESP_ERR_INVALID_CRC;
    }

    // check humidity crc
    if (crc8(raw_data + 3, 2) != raw_data[5])
    {
        ESP_LOGE(TAG, "CRC check for humidity data failed");
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t sht3x_init_desc(sht3x_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t sht3x_free_desc(sht3x_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sht3x_init(sht3x_t *dev)
{
    CHECK_ARG(dev);

    dev->mode = SHT3X_SINGLE_SHOT;
    dev->meas_start_time = 0;
    dev->meas_started = false;
    dev->meas_first = false;

    return send_cmd(dev, SHT3X_CLEAR_STATUS_CMD);
}

esp_err_t sht3x_set_heater(sht3x_t *dev, bool enable)
{
    CHECK_ARG(dev);

    return send_cmd(dev, enable ? SHT3X_HEATER_ON_CMD : SHT3X_HEATER_OFF_CMD);
}

esp_err_t sht3x_compute_values(sht3x_raw_data_t raw_data, float *temperature, float *humidity)
{
    CHECK_ARG(raw_data && (temperature || humidity));

    if (temperature)
        *temperature = ((((raw_data[0] * 256.0) + raw_data[1]) * 175) / 65535.0) - 45;

    if (humidity)
        *humidity = ((((raw_data[3] * 256.0) + raw_data[4]) * 100) / 65535.0);

    return ESP_OK;
}

esp_err_t sht3x_measure(sht3x_t *dev, float *temperature, float *humidity)
{
    CHECK_ARG(dev && (temperature || humidity));

    sht3x_raw_data_t raw_data;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, start_nolock(dev, SHT3X_SINGLE_SHOT, SHT3X_HIGH));
    vTaskDelay(SHT3X_MEAS_DURATION_TICKS[SHT3X_HIGH]);
    I2C_DEV_CHECK(&dev->i2c_dev, get_raw_data_nolock(dev, raw_data));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return sht3x_compute_values(raw_data, temperature, humidity);
}

uint8_t sht3x_get_measurement_duration(sht3x_repeat_t repeat)
{
    return SHT3X_MEAS_DURATION_TICKS[repeat];  // in RTOS ticks
}

esp_err_t sht3x_start_measurement(sht3x_t *dev, sht3x_mode_t mode, sht3x_repeat_t repeat)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, start_nolock(dev, mode, repeat));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t sht3x_stop_periodic_measurement(sht3x_t *dev)
{
    CHECK_ARG(dev);

    CHECK(send_cmd(dev, SHT3X_STOP_PERIODIC_MEAS_CMD));
    dev->mode = SHT3X_SINGLE_SHOT;
    dev->meas_start_time = 0;
    dev->meas_started = false;
    dev->meas_first = false;

    return ESP_OK;
}

esp_err_t sht3x_get_raw_data(sht3x_t *dev, sht3x_raw_data_t raw_data)
{
    CHECK_ARG(dev && raw_data);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, get_raw_data_nolock(dev, raw_data));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t sht3x_get_results(sht3x_t *dev, float *temperature, float *humidity)
{
    CHECK_ARG(dev && (temperature || humidity));

    sht3x_raw_data_t raw_data;

    CHECK(sht3x_get_raw_data(dev, raw_data));

    return sht3x_compute_values(raw_data, temperature, humidity);
}

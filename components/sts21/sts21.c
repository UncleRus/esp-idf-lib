/*
 * Copyright (c) 2022 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file sts21.c
 *
 * ESP-IDF driver for humidty/temperature sensors STS2110/STS2115/STS2120
 *
 * Copyright (c) 2022 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "sts21.h"
#include <ets_sys.h>
#include <esp_log.h>

static const char *TAG = "sts21";

#define I2C_FREQ_HZ 400000 // 400kHz
#define I2C_ADDRESS 0x4a

#define CMD_TRIGGER_NO_HOLD 0xf3
#define CMD_WRITE_USER_REG  0xe6
#define CMD_READ_USER_REG   0xe7
#define CMD_SOFT_RESET      0xfe

#define BIT_RES_H        7
#define BIT_BATTERY_FAIL 6
#define BIT_HEATER       2
#define BIT_RES_L        0

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const uint32_t measurement_time_us[] = {
    [STS21_RESOLUTION_14] = 85000,
    [STS21_RESOLUTION_13] = 43000,
    [STS21_RESOLUTION_12] = 22000,
    [STS21_RESOLUTION_11] = 11000,
};

static bool check_crc(uint16_t value, uint8_t crc)
{
    uint32_t row = (uint32_t)value << 8;
    row |= crc;

    uint32_t divisor = 0x988000;

    for (int i = 0; i < 16; i++)
    {
        if (row & (uint32_t)1 << (23 - i))
            row ^= divisor;
        divisor >>= 1;
    }

    return !row;
}

static esp_err_t update_user_reg(sts21_t *dev, uint8_t value, uint8_t mask)
{
    uint8_t r;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, CMD_READ_USER_REG, &r, 1));
    r = (r & ~mask) | value;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, CMD_WRITE_USER_REG, &r, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_user_reg(sts21_t *dev, uint8_t *value)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, CMD_READ_USER_REG, value, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t write_cmd(sts21_t *dev, uint8_t cmd)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t sts21_init_desc(sts21_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = I2C_ADDRESS;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t sts21_free_desc(sts21_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sts21_init(sts21_t *dev)
{
    CHECK_ARG(dev);

    // soft reset
    CHECK(write_cmd(dev, CMD_SOFT_RESET));
    dev->resolution = STS21_RESOLUTION_14;

    return sts21_set_heater_state(dev, false);
}

esp_err_t sts21_get_resolution(sts21_t *dev, sts21_resolution_t *res)
{
    CHECK_ARG(dev && res);

    uint8_t r;
    CHECK(read_user_reg(dev, &r));
    dev->resolution = *res = ((r >> BIT_RES_H) << 1) | (r & BIT(BIT_RES_L));

    return ESP_OK;
}

esp_err_t sts21_set_resolution(sts21_t *dev, sts21_resolution_t res)
{
    CHECK_ARG(dev);

    CHECK(update_user_reg(dev, (((res >> 1) & 1) << BIT_RES_H) | (res & 1), BIT(BIT_RES_H) | BIT(BIT_RES_L)));
    dev->resolution = res;

    return ESP_OK;
}

esp_err_t sts21_get_heater_state(sts21_t *dev, bool *on)
{
    CHECK_ARG(dev && on);

    uint8_t r;
    CHECK(read_user_reg(dev, &r));
    *on = r & BIT(BIT_HEATER) ? true : false;

    return ESP_OK;
}

esp_err_t sts21_set_heater_state(sts21_t *dev, bool on)
{
    CHECK_ARG(dev);

    return update_user_reg(dev, on ? BIT(BIT_HEATER) : 0, BIT(BIT_HEATER));
}

esp_err_t sts21_get_power_alert(sts21_t *dev, bool *alert)
{
    CHECK_ARG(dev && alert);

    uint8_t r;
    CHECK(read_user_reg(dev, &r));
    *alert = r & BIT(BIT_BATTERY_FAIL) ? true : false;

    return ESP_OK;
}

esp_err_t sts21_is_busy(sts21_t *dev, bool *busy)
{
    CHECK_ARG(dev && busy);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    esp_err_t res = i2c_dev_probe(&dev->i2c_dev, I2C_DEV_READ);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (res == ESP_FAIL)
    {
        // sensor NACKs I2C read operation when measurement is in progress
        *busy = true;
        return ESP_OK;
    }

    if (res == ESP_OK)
    {
        *busy = false;
        return ESP_OK;
    }

    return res;
}

esp_err_t sts21_trigger_measurement(sts21_t *dev)
{
    CHECK_ARG(dev);

    return write_cmd(dev, CMD_TRIGGER_NO_HOLD);
}

esp_err_t sts21_read_temperature(sts21_t *dev, float *t)
{
    CHECK_ARG(dev && t);

    uint8_t buf[3];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, buf, sizeof(buf)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];

    // calc crc
    if (!check_crc(raw, buf[2]))
    {
        ESP_LOGE(TAG, "Invalid CRC. Raw data: 0x%04x, CRC: 0x%02x", raw, buf[2]);
        return ESP_ERR_INVALID_CRC;
    }

    raw &= 0xffff << (dev->resolution + 2);
    // calc temperature
    *t = (-46.85f + (175.72f * raw / 65536.0f)) * 1.8f + 32;

    return ESP_OK;
}

esp_err_t sts21_measure(sts21_t *dev, float *t)
{
    CHECK(sts21_trigger_measurement(dev));
    ets_delay_us(measurement_time_us[dev->resolution]);
    return sts21_read_temperature(dev, t);
}

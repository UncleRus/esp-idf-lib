/*
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
 * @file si7021.c
 *
 * ESP-IDF driver for Si7013/Si7020/Si7021/HTU2xD/SHT2x and
 * compatible temperature and humidity sensors
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include "si7021.h"

#define I2C_FREQ_HZ 400000 // 400kHz

static const char *TAG = "si7021";

#define DELAY_MS 100  // fixed delay (100 ms for SHT20)

#define CMD_MEAS_RH_HOLD     0xe5 // not used, can't stretch clock
#define CMD_MEAS_RH_NOHOLD   0xf5
#define CMD_MEAS_T_HOLD      0xe3 // not used, can't stretch clock
#define CMD_MEAS_T_NOHOLD    0xf3
#define CMD_READ_T           0xe0 // not used
#define CMD_RESET            0xfe
#define CMD_WRITE_USER_REG   0xe6
#define CMD_READ_USER_REG    0xe7
#define CMD_WRITE_HEATER_REG 0x51
#define CMD_READ_HEATER_REG  0x11
#define CMD_READ_ID_1        0x0ffa
#define CMD_READ_ID_2        0xc9fc
#define CMD_READ_FW_REV_1    0xb884

#define BIT_USER_REG_RES0 0
#define BIT_USER_REG_HTRE 2
#define BIT_USER_REG_RES1 7

#define HEATER_MASK 0x0f

#define BV(x) (1 << (x))

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

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

static esp_err_t measure(i2c_dev_t *dev, uint8_t cmd, uint16_t *raw)
{
    I2C_DEV_TAKE_MUTEX(dev);
    // write command
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &cmd, 1));

    // wait
    vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);

    // read data
    uint8_t buf[3];
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, buf, 3));
    I2C_DEV_GIVE_MUTEX(dev);

    *raw = ((uint16_t)buf[0] << 8) | buf[1];

    if (!check_crc(*raw, buf[2]))
    {
        ESP_LOGE(TAG, "Invalid CRC");
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t si7021_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = SI7021_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t si7021_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t si7021_reset(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    uint8_t cmd = CMD_RESET;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &cmd, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    vTaskDelay(pdMS_TO_TICKS(DELAY_MS));

    return ESP_OK;
}

esp_err_t si7021_get_heater(i2c_dev_t *dev, bool *on)
{
    CHECK_ARG(dev && on);

    uint8_t u;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, CMD_READ_USER_REG, &u, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *on = u & BV(BIT_USER_REG_HTRE);

    return ESP_OK;
}

esp_err_t si7021_set_heater(i2c_dev_t *dev, bool on)
{
    CHECK_ARG(dev);

    uint8_t u;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, CMD_READ_USER_REG, &u, 1));
    u = (u & ~BV(BIT_USER_REG_HTRE)) | (on ? 1 : 0);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, CMD_WRITE_USER_REG, &u, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t si7021_get_heater_current(i2c_dev_t *dev, uint8_t *level)
{
    CHECK_ARG(dev && level);

    uint8_t h;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, CMD_READ_HEATER_REG, &h, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *level = h & HEATER_MASK;

    return ESP_OK;
}

esp_err_t si7021_set_heater_current(i2c_dev_t *dev, uint8_t level)
{
    CHECK_ARG(dev && level <= SI7021_MAX_HEATER_CURRENT);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, CMD_WRITE_HEATER_REG, &level, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t si7021_get_resolution(i2c_dev_t *dev, si7021_resolution_t *r)
{
    CHECK_ARG(dev && r);

    uint8_t u;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, CMD_READ_USER_REG, &u, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *r = ((u >> BIT_USER_REG_RES1) << 1) | (u & BV(BIT_USER_REG_RES0));

    return ESP_OK;
}

esp_err_t si7021_set_resolution(i2c_dev_t *dev, si7021_resolution_t r)
{
    CHECK_ARG(dev && r <= SI7021_RES_RH11_T11);

    uint8_t u;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, CMD_READ_USER_REG, &u, 1));
    u = (u & ~(BV(BIT_USER_REG_RES0) | BV(BIT_USER_REG_RES1))) | ((r & 2) << (BIT_USER_REG_RES1 - 1)) | (r & 1);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, CMD_WRITE_USER_REG, &u, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t si7021_measure_temperature(i2c_dev_t *dev, float *t)
{
    CHECK_ARG(dev && t);

    uint16_t raw;
    CHECK(measure(dev, CMD_MEAS_T_NOHOLD, &raw));
    *t = raw * 175.72 / 65536 - 46.85;

    return ESP_OK;
}

esp_err_t si7021_measure_humidity(i2c_dev_t *dev, float *rh)
{
    CHECK_ARG(dev && rh);

    uint16_t raw;
    CHECK(measure(dev, CMD_MEAS_RH_NOHOLD, &raw));
    *rh = raw * 125.0 / 65536 - 6;

    return ESP_OK;
}

esp_err_t si7021_get_serial(i2c_dev_t *dev, uint64_t *serial, bool sht2x_mode)
{
    CHECK_ARG(dev && serial);

    uint8_t buf[8];
    uint16_t cmd;

    I2C_DEV_TAKE_MUTEX(dev);

    // read SNA, ignore CRC
    cmd = CMD_READ_ID_1;
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, &cmd, 2, buf, 8));
    *serial = sht2x_mode
        ? ((uint64_t)buf[0] << 40) | ((uint64_t)buf[2] << 32) | ((uint64_t)buf[4] << 24) | ((uint64_t)buf[6] << 16)
        : ((uint64_t)buf[0] << 56) | ((uint64_t)buf[2] << 48) | ((uint64_t)buf[4] << 40) | ((uint64_t)buf[6] << 32);

    // read SNB, ignore CRC
    cmd = CMD_READ_ID_2;
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, &cmd, 2, buf, 6));
    *serial |= sht2x_mode
        ? ((uint32_t)buf[0] << 8) | ((uint64_t)buf[3] << 56) | ((uint64_t)buf[4] << 48) | buf[1]
        : ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[3] << 8) | buf[4];

    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t si7021_get_device_id(i2c_dev_t *dev, si7021_device_id_t *id)
{
    CHECK_ARG(id);

    uint64_t serial;
    CHECK(si7021_get_serial(dev, &serial, false));

    switch ((serial >> 24) & 0xff)
    {
        case 0x0d:
            *id = SI_MODEL_SI7013;
            break;
        case 0x14:
            *id = SI_MODEL_SI7020;
            break;
        case 0x15:
            *id = SI_MODEL_SI7021;
            break;
        case 0x00:
        case 0xff:
            *id = SI_MODEL_SAMPLE;
            break;
        default:
            *id = SI_MODEL_UNKNOWN;
    }

    return ESP_OK;
}

esp_err_t si7021_get_device_revision(i2c_dev_t *dev, uint8_t *rev)
{
    CHECK_ARG(dev && rev);

    uint16_t cmd = CMD_READ_FW_REV_1;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, &cmd, 2, rev, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

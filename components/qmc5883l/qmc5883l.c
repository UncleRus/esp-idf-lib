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
 * @file qmc5883l.c
 *
 * ESP-IDF Driver for 3-axis magnetic sensor QMC5883L
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "qmc5883l.h"

#define I2C_FREQ_HZ 400000 // 400kHz

#define REG_XOUT_L 0x00
#define REG_XOUT_H 0x01
#define REG_YOUT_L 0x02
#define REG_YOUT_H 0x03
#define REG_ZOUT_L 0x04
#define REG_ZOUT_H 0x05
#define REG_STATE  0x06
#define REG_TOUT_L 0x07
#define REG_TOUT_H 0x08
#define REG_CTRL1  0x09
#define REG_CTRL2  0x0a
#define REG_FBR    0x0b
#define REG_ID     0x0d

#define MASK_MODE  0xfe
#define MASK_ODR   0xf3

static const char *TAG = "qmc5883l";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

inline static esp_err_t write_reg_nolock(qmc5883l_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1);
}

inline static esp_err_t read_reg_nolock(qmc5883l_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(&dev->i2c_dev, reg, val, 1);
}

static esp_err_t write_reg(qmc5883l_t *dev, uint8_t reg, uint8_t val)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_reg(qmc5883l_t *dev, uint8_t reg, uint8_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t qmc5883l_init_desc(qmc5883l_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
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

esp_err_t qmc5883l_free_desc(qmc5883l_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t qmc5883l_reset(qmc5883l_t *dev)
{
    CHECK(write_reg(dev, REG_CTRL2, 0x80));
    dev->range = QMC5883L_RNG_2;

    return ESP_OK;
}

esp_err_t qmc5883l_get_chip_id(qmc5883l_t *dev, uint8_t *id)
{
    return read_reg(dev, REG_ID, id);
}

esp_err_t qmc5883l_set_mode(qmc5883l_t *dev, qmc5883l_mode_t mode)
{
    CHECK_ARG(dev && mode <= QMC5883L_MODE_CONTINUOUS);

    uint8_t v;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, REG_CTRL1, &v));
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, REG_CTRL1, (v & 0xfe) | mode));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t qmc5883l_get_mode(qmc5883l_t *dev, qmc5883l_mode_t *mode)
{
    CHECK_ARG(mode);

    uint8_t v;
    CHECK(read_reg(dev, REG_CTRL1, &v));
    *mode = v & 1;

    return ESP_OK;
}

esp_err_t qmc5883l_set_config(qmc5883l_t *dev, qmc5883l_odr_t odr, qmc5883l_osr_t osr, qmc5883l_range_t rng)
{
    CHECK_ARG(dev && odr <= QMC5883L_DR_200 && osr <= QMC5883L_OSR_512 && rng <= QMC5883L_RNG_8);

    uint8_t v;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, REG_CTRL1, &v));
    dev->range = rng;
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, REG_FBR, 1)); // Define set/reset period
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, REG_CTRL1,
            (v & 0x03) | ((odr & 3) << 2) | ((rng & 1) << 4) | ((osr & 3) << 6)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t qmc5883l_get_config(qmc5883l_t *dev, qmc5883l_odr_t *odr, qmc5883l_osr_t *osr, qmc5883l_range_t *rng)
{
    CHECK_ARG(odr && osr && rng);

    uint8_t v;
    CHECK(read_reg(dev, REG_CTRL1, &v));
    *odr = (v >> 2) & 3;
    *osr = (v >> 6) & 3;
    *rng = (v >> 4) & 1;

    return ESP_OK;
}

esp_err_t qmc5883l_set_int(qmc5883l_t *dev, bool enable)
{
    return write_reg(dev, REG_CTRL2, enable ? 1 : 0);
}

esp_err_t qmc5883l_get_int(qmc5883l_t *dev, bool *enable)
{
    CHECK_ARG(enable);

    uint8_t v;
    CHECK(read_reg(dev, REG_CTRL2, &v));
    *enable = v & 1;

    return ESP_OK;
}

esp_err_t qmc5883l_data_ready(qmc5883l_t *dev, bool *ready)
{
    CHECK_ARG(ready);

    uint8_t v;
    CHECK(read_reg(dev, REG_STATE, &v));
    *ready = v & 1;

    return ESP_OK;
}

esp_err_t qmc5883l_get_raw_data(qmc5883l_t *dev, qmc5883l_raw_data_t *raw)
{
    CHECK_ARG(dev && raw);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev, REG_XOUT_L, raw, 6);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not read data register, err = %d", ret);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;
}

esp_err_t qmc5883l_raw_to_mg(qmc5883l_t *dev, qmc5883l_raw_data_t *raw, qmc5883l_data_t *data)
{
    CHECK_ARG(dev && raw && data);

    float f = (dev->range == QMC5883L_RNG_2 ? 2000.0 : 8000.0) / 32768;

    data->x = raw->x * f;
    data->y = raw->y * f;
    data->z = raw->z * f;

    return ESP_OK;
}

esp_err_t qmc5883l_get_data(qmc5883l_t *dev, qmc5883l_data_t *data)
{
    qmc5883l_raw_data_t raw;
    CHECK(qmc5883l_get_raw_data(dev, &raw));
    return qmc5883l_raw_to_mg(dev, &raw, data);
}

esp_err_t qmc5883l_get_raw_temp(qmc5883l_t *dev, int16_t *temp)
{
    CHECK_ARG(dev && temp);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev, REG_TOUT_L, temp, 2);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not read TOUT register, err = %d", ret);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;
}

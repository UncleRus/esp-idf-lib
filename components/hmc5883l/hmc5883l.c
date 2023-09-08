/*
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file hmc5883l.c
 *
 * ESP-IDF Driver for 3-axis digital compass HMC5883L and HMC5983L
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "hmc5883l.h"
#include <inttypes.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_idf_lib_helpers.h>

#define REG_CR_A 0x00
#define REG_CR_B 0x01
#define REG_MODE 0x02
#define REG_DX_H 0x03
#define REG_DX_L 0x04
#define REG_DZ_H 0x05
#define REG_DZ_L 0x06
#define REG_DY_H 0x07
#define REG_DY_L 0x08
#define REG_STAT 0x09
#define REG_ID_A 0x0a
#define REG_ID_B 0x0b
#define REG_ID_C 0x0c

#define BIT_MA  5
#define BIT_DO  2
#define BIT_GN  5

#define MASK_MD 0x03
#define MASK_MA 0x60
#define MASK_DO 0x1c
#define MASK_MS 0x03
#define MASK_DR 0x01
#define MASK_DL 0x02

#define I2C_FREQ_HZ 400000

static const char *TAG = "hmc5883l";

static const float gain_values [] = {
    [HMC5883L_GAIN_1370] = 0.73,
    [HMC5883L_GAIN_1090] = 0.92,
    [HMC5883L_GAIN_820]  = 1.22,
    [HMC5883L_GAIN_660]  = 1.52,
    [HMC5883L_GAIN_440]  = 2.27,
    [HMC5883L_GAIN_390]  = 2.56,
    [HMC5883L_GAIN_330]  = 3.03,
    [HMC5883L_GAIN_230]  = 4.35
};

#define timeout_expired(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t hmc5883l_init_desc(hmc5883l_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = HMC5883L_ADDR;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t hmc5883l_free_desc(hmc5883l_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

static esp_err_t write_register(hmc5883l_dev_t *dev, uint8_t reg, uint8_t val)
{
    esp_err_t ret = i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not write 0x%02x to register 0x%02x, err = %d", val, reg, ret);
    return ret;
}

static inline esp_err_t read_register(hmc5883l_dev_t *dev, uint8_t reg, uint8_t *val)
{
    esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev, reg, val, 1);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not read register 0x%02x, err = %d", reg, ret);
    return ret;
}

static esp_err_t update_register(hmc5883l_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t old;
    esp_err_t ret = read_register(dev, reg, &old);
    if (ret != ESP_OK)
        return ret;
    return write_register(dev, reg, (old & mask) | val);
}

#define CHECK(X) do { \
        esp_err_t __ = X; \
        if (__ != ESP_OK) return __; \
    } while (0)

esp_err_t hmc5883l_init(hmc5883l_dev_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint32_t id = 0;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_ID_A, &id, 3));
    if (id != HMC5883L_ID)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        ESP_LOGE(TAG, "Unknown ID: 0x%08" PRIx32 " != 0x%08x", id, HMC5883L_ID);
        return ESP_ERR_NOT_FOUND;
    }

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    hmc5883l_gain_t gain;
    CHECK(hmc5883l_get_gain(dev, &gain));
    dev->gain = gain_values[gain];

    CHECK(hmc5883l_get_opmode(dev, &dev->opmode));

    return ESP_OK;
}

esp_err_t hmc5883l_get_opmode(hmc5883l_dev_t *dev, hmc5883l_opmode_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_MODE, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val = (*val & MASK_MD) == 0 ? HMC5883L_MODE_CONTINUOUS : HMC5883L_MODE_SINGLE;
    return ESP_OK;
}

esp_err_t hmc5883l_set_opmode(hmc5883l_dev_t *dev, hmc5883l_opmode_t mode)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, REG_MODE, mode));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->opmode = mode;
    return ESP_OK;
}

esp_err_t hmc5883l_get_samples_averaged(hmc5883l_dev_t *dev, hmc5883l_samples_averaged_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_CR_A, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val = (*val & MASK_MA) >> BIT_MA;
    return ESP_OK;
}

esp_err_t hmc5883l_set_samples_averaged(hmc5883l_dev_t *dev, hmc5883l_samples_averaged_t samples)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, update_register(dev, REG_CR_A, MASK_MA, samples << BIT_MA));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t hmc5883l_get_data_rate(hmc5883l_dev_t *dev, hmc5883l_data_rate_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_CR_A, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val = (*val & MASK_DO) >> BIT_DO;
    return ESP_OK;
}

esp_err_t hmc5883l_set_data_rate(hmc5883l_dev_t *dev, hmc5883l_data_rate_t rate)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, update_register(dev, REG_CR_A, MASK_DO, rate << BIT_DO));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t hmc5883l_get_bias(hmc5883l_dev_t *dev, hmc5883l_bias_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_CR_A, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val &= MASK_MS;
    return ESP_OK;
}

esp_err_t hmc5883l_set_bias(hmc5883l_dev_t *dev, hmc5883l_bias_t bias)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, update_register(dev, REG_CR_A, MASK_MS, bias));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t hmc5883l_get_gain(hmc5883l_dev_t *dev, hmc5883l_gain_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_CR_B, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val >>= BIT_GN;
    return ESP_OK;
}

esp_err_t hmc5883l_set_gain(hmc5883l_dev_t *dev, hmc5883l_gain_t gain)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, REG_CR_B, gain << BIT_GN));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->gain = gain_values[gain];
    return ESP_OK;
}

esp_err_t hmc5883l_data_is_locked(hmc5883l_dev_t *dev, bool *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_STAT, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val &= MASK_DL;
    return ESP_OK;
}

esp_err_t hmc5883l_data_is_ready(hmc5883l_dev_t *dev, bool *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_STAT, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *val &= MASK_DR;
    return ESP_OK;
}

esp_err_t hmc5883l_get_raw_data(hmc5883l_dev_t *dev, hmc5883l_raw_data_t *data)
{
    CHECK_ARG(dev && data);

    if (dev->opmode == HMC5883L_MODE_SINGLE)
    {
        // overwrite mode register for measurement
        CHECK(hmc5883l_set_opmode(dev, dev->opmode));
        // wait for data
        uint64_t start = esp_timer_get_time();
        bool dready = false;
        do
        {
            CHECK(hmc5883l_data_is_ready(dev, &dready));
            if (timeout_expired(start, CONFIG_HMC5883L_MEAS_TIMEOUT))
                return ESP_ERR_TIMEOUT;
        } while (!dready);
    }
    uint8_t buf[6];
    uint8_t reg = REG_DX_H;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, buf, 6));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    data->x = ((int16_t)buf[REG_DX_H - REG_DX_H] << 8) | buf[REG_DX_L - REG_DX_H];
    data->y = ((int16_t)buf[REG_DY_H - REG_DX_H] << 8) | buf[REG_DY_L - REG_DX_H];
    data->z = ((int16_t)buf[REG_DZ_H - REG_DX_H] << 8) | buf[REG_DZ_L - REG_DX_H];

    return ESP_OK;
}

esp_err_t hmc5883l_raw_to_mg(const hmc5883l_dev_t *dev, const hmc5883l_raw_data_t *raw, hmc5883l_data_t *mg)
{
    CHECK_ARG(dev && raw && mg);

    mg->x = raw->x * dev->gain;
    mg->y = raw->y * dev->gain;
    mg->z = raw->z * dev->gain;

    return ESP_OK;
}

esp_err_t hmc5883l_get_data(hmc5883l_dev_t *dev, hmc5883l_data_t *data)
{
    CHECK_ARG(data);

    hmc5883l_raw_data_t raw;

    CHECK(hmc5883l_get_raw_data(dev, &raw));
    CHECK(hmc5883l_raw_to_mg(dev, &raw, data));

    return ESP_OK;
}

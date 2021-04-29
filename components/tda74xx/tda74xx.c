/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file tda74xx.c
 *
 * ESP-IDF driver for TDA7439/TDA7439DS/TDA7440 audioprocessors
 *
 * Copyright (c) 2018, 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "tda74xx.h"

#define I2C_FREQ_HZ 100000 // 100kHz

static const char *TAG = "tda74xx";

#define REG_INPUT_SELECTOR 0x00
#define REG_INPUT_GAIN     0x01
#define REG_VOLUME         0x02
#define REG_BASS_GAIN      0x03
#define REG_MID_GAIN       0x04
#define REG_TREBLE_GAIN    0x05
#define REG_ATTEN_R        0x06
#define REG_ATTEN_L        0x07

#define MUTE_VALUE 0x38

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    ESP_LOGD(TAG, "%02x -> %02x", val, reg);

    return ESP_OK;
}

static esp_err_t read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    ESP_LOGD(TAG, "%02x <- %02x", *val, reg);

    return ESP_OK;
}

esp_err_t tda74xx_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = TDA74XX_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t tda74xx_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t tda74xx_set_input(i2c_dev_t *dev, uint8_t input)
{
    CHECK_ARG(dev && input <= TDA74XX_MAX_INPUT);

    return write_reg(dev, REG_INPUT_SELECTOR, input);
}

esp_err_t tda74xx_get_input(i2c_dev_t *dev, uint8_t *input)
{
    CHECK_ARG(dev && input);

    return read_reg(dev, REG_INPUT_SELECTOR, input);
}

esp_err_t tda74xx_set_input_gain(i2c_dev_t *dev, uint8_t gain_db)
{
    CHECK_ARG(dev && gain_db <= TDA74XX_MAX_INPUT_GAIN);

    return write_reg(dev, REG_INPUT_GAIN, gain_db / 2);
}

esp_err_t tda74xx_get_input_gain(i2c_dev_t *dev, uint8_t *gain_db)
{
    CHECK_ARG(dev && gain_db);

    CHECK(read_reg(dev, REG_INPUT_GAIN, gain_db));
    *gain_db *= 2;

    return ESP_OK;
}

esp_err_t tda74xx_set_volume(i2c_dev_t *dev, int8_t volume_db)
{
    CHECK_ARG(dev && volume_db <= TDA74XX_MAX_VOLUME && volume_db >= TDA74XX_MIN_VOLUME);

    return write_reg(dev, REG_VOLUME, volume_db == TDA74XX_MIN_VOLUME ? MUTE_VALUE : -volume_db);
}

esp_err_t tda74xx_get_volume(i2c_dev_t *dev, int8_t *volume_db)
{
    CHECK_ARG(dev && volume_db);

    CHECK(read_reg(dev, REG_VOLUME, (uint8_t *)volume_db));
    *volume_db = -*volume_db;
    return ESP_OK;
}

esp_err_t tda74xx_set_equalizer_gain(i2c_dev_t *dev, tda74xx_band_t band, int8_t gain_db)
{
    CHECK_ARG(dev && gain_db >= TDA74XX_MIN_EQ_GAIN && gain_db <= TDA74XX_MAX_EQ_GAIN);

    uint8_t reg;
    switch(band)
    {
        case TDA74XX_BAND_BASS:
            reg = REG_BASS_GAIN;
            break;
        case TDA74XX_BAND_MIDDLE:
            reg = REG_MID_GAIN;
            break;
        default:
            reg = REG_TREBLE_GAIN;
    }

    return write_reg(dev, reg, (gain_db + 14) / 2);
}

esp_err_t tda74xx_get_equalizer_gain(i2c_dev_t *dev, tda74xx_band_t band, int8_t *gain_db)
{
    CHECK_ARG(dev && gain_db);

    uint8_t reg;
    switch(band)
    {
        case TDA74XX_BAND_BASS:
            reg = REG_BASS_GAIN;
            break;
        case TDA74XX_BAND_MIDDLE:
            reg = REG_MID_GAIN;
            break;
        default:
            reg = REG_TREBLE_GAIN;
    }

    CHECK(read_reg(dev, reg, (uint8_t *)gain_db));
    *gain_db = *gain_db * 2 - 14;
    return ESP_OK;
}

esp_err_t tda74xx_set_speaker_attenuation(i2c_dev_t *dev, tda74xx_channel_t channel, uint8_t atten_db)
{
    CHECK_ARG(dev && atten_db <= TDA74XX_MAX_ATTEN);

    return write_reg(dev, channel == TDA74XX_CHANNEL_LEFT ? REG_ATTEN_L : REG_ATTEN_R, atten_db);
}

esp_err_t tda74xx_get_speaker_attenuation(i2c_dev_t *dev, tda74xx_channel_t channel, uint8_t *atten_db)
{
    CHECK_ARG(dev && atten_db);

    return read_reg(dev, channel == TDA74XX_CHANNEL_LEFT ? REG_ATTEN_L : REG_ATTEN_R, atten_db);
}

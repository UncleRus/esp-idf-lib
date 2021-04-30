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
 * @file tda74xx.h
 * @defgroup tda74xx tda74xx
 * @{
 *
 * ESP-IDF driver for TDA7439/TDA7439DS/TDA7440 audioprocessors
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TDA74XX_H__
#define __TDA74XX_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TDA74XX_ADDR 0x44 //!< I2C address

#define TDA74XX_MAX_INPUT      3    //!< Maximum input number

#define TDA74XX_MIN_VOLUME     -48  //!< Mute volume level, dB
#define TDA74XX_MAX_VOLUME     0    //!< Maximum colume level, dB

#define TDA74XX_MAX_INPUT_GAIN 30   //!< Maximum input gain, dB

#define TDA74XX_MIN_EQ_GAIN    -14  //!< Minimum equalizer gain, dB
#define TDA74XX_MAX_EQ_GAIN    14   //!< Maximum equalizer gain, dB

#define TDA74XX_MAX_ATTEN      56   //!< Maximum speaker attenuation level, dB

/**
 * Audio channel
 */
typedef enum {
    TDA74XX_CHANNEL_LEFT = 0,
    TDA74XX_CHANNEL_RIGHT
} tda74xx_channel_t;

/**
 * Equalizer band
 */
typedef enum {
    TDA74XX_BAND_BASS = 0,
    TDA74XX_BAND_MIDDLE,    //!< Not supported on TDA7440
    TDA74XX_BAND_TREBLE,
} tda74xx_band_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_free_desc(i2c_dev_t *dev);

/**
 * @brief Switch input
 *
 * @param dev Device descriptor
 * @param input Input #, 0..3
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_input(i2c_dev_t *dev, uint8_t input);

/**
 * @brief Get current input
 *
 * @param dev Device descriptor
 * @param[out] input Input #, 0..3
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_get_input(i2c_dev_t *dev, uint8_t *input);

/**
 * @brief Set input gain, dB
 *
 * @param dev Device descriptor
 * @param gain_db Gain, 0..30 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_input_gain(i2c_dev_t *dev, uint8_t gain_db);

/**
 * @brief Get input gain
 *
 * @param dev Device descriptor
 * @param[out] gain_db Gain, 0..30 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_get_input_gain(i2c_dev_t *dev, uint8_t *gain_db);

/**
 * @brief Set master volume
 *
 * @param dev Device descriptor
 * @param volume_db Volume, -48..0 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_volume(i2c_dev_t *dev, int8_t volume_db);

/**
 * @brief Get master volume
 *
 * @param dev Device descriptor
 * @param[out] volume_db Volume, -48..0 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_get_volume(i2c_dev_t *dev, int8_t *volume_db);

/**
 * @brief Set equalizer gain
 *
 * @param dev Device descriptor
 * @param band Band
 * @param gain_db Gain, -14..14 dB in 2 dB step
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_equalizer_gain(i2c_dev_t *dev, tda74xx_band_t band, int8_t gain_db);

/**
 * @brief Get equalizer gain
 *
 * @param dev Device descriptor
 * @param band Band
 * @param[out] gain_db Gain, -14..14 dB in 2 dB step
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_get_equalizer_gain(i2c_dev_t *dev, tda74xx_band_t band, int8_t *gain_db);

/**
 * @brief Attenuate speaker
 *
 * @param dev Device descriptor
 * @param channel Audio channel
 * @param atten_db Attenuation, 0..56 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_speaker_attenuation(i2c_dev_t *dev, tda74xx_channel_t channel, uint8_t atten_db);

/**
 * @brief Get speaker attenuation
 *
 * @param dev Device descriptor
 * @param channel Audio channel
 * @param[out] atten_db Attenuation, 0..56 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_get_speaker_attenuation(i2c_dev_t *dev, tda74xx_channel_t channel, uint8_t *atten_db);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __TDA74XX_H__ */

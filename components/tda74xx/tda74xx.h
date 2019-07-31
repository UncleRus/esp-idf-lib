/**
 * @file tda74xx.h
 * @defgroup tda74xx tda74xx
 * @{
 *
 * ESP-IDF driver for TDA7439/TDA7439DS/TDA7440 audioprocessors
 *
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TDA74XX_H__
#define __TDA74XX_H__

#include <stdbool.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TDA74XX_ADDR 0x44 //!< I2C address

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
 * Initialize device descriptor
 * @param dev Device descriptor
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_free_desc(i2c_dev_t *dev);

/**
 * Switch input
 * @param dev Device descriptor
 * @param input Input #, 0..3
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_input(i2c_dev_t *dev, uint8_t input);

/**
 * Set input gain
 * @param dev Device descriptor
 * @param gain_db Gain, 0..30 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_input_gain(i2c_dev_t *dev, uint8_t gain_db);

/**
 * Set master volume
 * @param dev Device descriptor
 * @param volume_db Volume, -48..0 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_volume(i2c_dev_t *dev, int8_t volume_db);

/**
 * Set equalizer gain
 * @param dev Device descriptor
 * @param band Band
 * @param gain_db Gain, -14..14 dB in 2 dB step
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_equalizer_gain(i2c_dev_t *dev, tda74xx_band_t band, int8_t gain_db);

/**
 * Attenuate speaker
 * @param dev Device descriptor
 * @param channel Audio channel
 * @param atten_db Attenuation, 0..56 dB
 * @return `ESP_OK` on success
 */
esp_err_t tda74xx_set_speaker_attenuation(i2c_dev_t *dev, tda74xx_channel_t channel, uint8_t atten_db);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __TDA74XX_H__ */

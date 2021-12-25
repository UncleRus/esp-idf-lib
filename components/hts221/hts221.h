/*
 * Copyright (c) 2021 saasaa <mail@saasaa.xyz>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file hts221.h
 * @brief HTS221 driver
 * @author saasaa
 * @defgroup hts221 hts221
 * @{
 *
 * ESP-IDF driver for HTS221 temperature and humidity sensor
 *
 * Datasheet: http://www.st.com/resource/en/datasheet/hts221.pdf
 * Technical note on interpreting humidity and temperature readings in the
 * HTS221 digital humidity sensor:
 * https://www.st.com/resource/en/technical_note/tn1218-interpreting-humidity-and-temperature-readings-in-the-hts221-digital-humidity-sensor-stmicroelectronics.pdf
 * Copyright (c) 2021 saasaa <mail@saasaa.xyz>
 *
 * ISC Licensed as described in the file LICENSE
 *
 */
#ifndef __HTS221_H__
#define __HTS221_H__

#include <esp_err.h>
#include <i2cdev.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HTS221_I2C_ADDRESS 0x5F

/**
 * @brief Struct for storing calibration parameters.
 *
 * Unique for every HTS221 Sensor.
 */
typedef struct
{
    uint8_t H0_rH;
    uint8_t H1_rH;
    uint16_t T0_degC;
    uint16_t T1_degC;
    int16_t H0_T0_OUT;
    int16_t H1_T0_OUT;
    int16_t T0_OUT;
    int16_t T1_OUT;
} hts221_calibration_param_t;

/**
 * @brief Number of averaged temperature samples
 */
typedef enum
{
    HTS221_AVGT_2 = 0,   //!< 2 averaged temperature samples
    HTS221_AVGT_4,       //!< 4 averaged temperature samples
    HTS221_AVGT_8,       //!< 8 averaged temperature samples
    HTS221_AVGT_1,       //!< 16 averaged temperature samples
    HTS221_AVGT_32,      //!< 32 averaged temperature samples
    HTS221_AVGT_64,      //!< 64 averaged temperature samples
    HTS221_AVGT_128,     //!< 128 averaged temperature samples
    HTS221_AVGT_256,     //!< 256 averaged temperature samples
} hts221_temperature_avg_t;

/**
 * @brief Number of averaged humidity samples
 */
typedef enum
{
    HTS221_AVGH_4 = 0,   //!< 4 averaged humidity samples
    HTS221_AVGH_8,       //!< 8 averaged humidity samples
    HTS221_AVGH_16,      //!< 16 averaged humidity samples
    HTS221_AVGH_32,      //!< 32 averaged humidity samples
    HTS221_AVGH_64,      //!< 64 averaged humidity samples
    HTS221_AVGH_128,     //!< 128 averaged humidity samples
    HTS221_AVGH_256,     //!< 256 averaged humidity samples
    HTS221_AVGH_512,     //!< 512 averaged humidity samples
} hts221_humidity_avg_t;

/**
 * @brief Output data rate
 */
typedef enum
{
    HTS221_ONE_SHOT = 0,
    HTS221_1HZ,
    HTS221_7HZ,
    HTS221_12_5HZ,
} hts221_data_rate_t;

typedef enum
{
    HTS221_DRDY_ACTIVE_HIGH = 0,
    HTS221_DRDY_ACTIVE_LOW,
} hts221_drdy_level_t;

typedef enum
{
    HTS221_DRDY_PUSH_PULL = 0,
    HTS221_DRDY_OPEN_DRAIN,
} hts221_drdy_mode_t;

typedef struct
{
    i2c_dev_t i2c_dev;
    hts221_calibration_param_t cal;
} hts221_t;

/**
 * @brief Initialize the HTS221 Device descriptor
 *
 * @param[out] dev Device descriptor
 * @param port     I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t hts221_init_desc(hts221_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hts221_free_desc(hts221_t *dev);

/**
 * @brief Reset device parameters, read calibration data
 *
 * Reboot device, reset calibration data
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hts221_init(hts221_t *dev);

/**
 * @brief Get power mode
 *
 * @param dev             Device descriptor
 * @param[out] power_down true if device in power down mode
 * @return `ESP_OK` on success
 */
esp_err_t hts221_get_power_mode(hts221_t *dev, bool *power_down);

/**
 * @brief Set power mode
 *
 * @param dev        Device descriptor
 * @param power_down true to set device to power down mode
 * @return `ESP_OK` on success
 */
esp_err_t hts221_set_power_mode(hts221_t *dev, bool power_down);

/**
 * @brief Get output data rate
 *
 * @param dev     Device descriptor
 * @param[out] dr Data rate
 * @return `ESP_OK` on success
 */
esp_err_t hts221_get_data_rate(hts221_t *dev, hts221_data_rate_t *dr);

/**
 * @brief Set output data rate
 *
 * @param dev Device descriptor
 * @param dr  Data rate
 * @return `ESP_OK` on success
 */
esp_err_t hts221_set_data_rate(hts221_t *dev, hts221_data_rate_t dr);

/**
 * @brief Get heater state
 *
 * @param dev         Device descriptor
 * @param[out] enable true when heater is enabled
 * @return `ESP_OK` on success
 */
esp_err_t hts221_get_heater(hts221_t *dev, bool *enable);

/**
 * @brief Switch heater on/off
 *
 * @param dev    Device descriptor
 * @param enable true to switch heater on
 * @return `ESP_OK` on success
 */
esp_err_t hts221_set_heater(hts221_t *dev, bool enable);

/**
 * @brief Get average configuration
 *
 * @param dev         Device descriptor
 * @param[out] t_avg  Temperature average configuration
 * @param[out] rh_avg Humidity average configuration
 * @return `ESP_OK` on success
 */
esp_err_t hts221_get_averaging(hts221_t *dev, hts221_temperature_avg_t *t_avg, hts221_humidity_avg_t *rh_avg);

/**
 * @brief Set average configuration
 *
 * @param dev    Device descriptor
 * @param t_avg  Temperature average configuration
 * @param rh_avg Humidity average configuration
 * @return `ESP_OK` on success
 */
esp_err_t hts221_set_averaging(hts221_t *dev, hts221_temperature_avg_t t_avg, hts221_humidity_avg_t rh_avg);

/**
 * @brief Get configuration of DRDY pin
 *
 * @param dev         Device descriptor
 * @param[out] enable true if DRDY pin is enabled
 * @param[out] mode   DRDY pin mode (Push-pull or open drain)
 * @param[out] active Pin active level
 * @return `ESP_OK` on success
 */
esp_err_t hts221_get_drdy_config(hts221_t *dev, bool *enable, hts221_drdy_mode_t *mode, hts221_drdy_level_t *active);

/**
 * @brief Set configuration of DRDY pin
 *
 * @param dev    Device descriptor
 * @param enable true if DRDY pin is enabled
 * @param mode   DRDY pin mode (Push-pull or open drain)
 * @param active Pin active level
 * @return `ESP_OK` on success
 */
esp_err_t hts221_set_drdy_config(hts221_t *dev, bool enable, hts221_drdy_mode_t mode, hts221_drdy_level_t active);

/**
 * @brief Check the availability of new RH/T data
 *
 * \p ready parameter will be true only if new data for both temperature and humidity is available.
 *
 * @param dev        Device descriptor
 * @param[out] ready true if new RH/T data available
 * @return `ESP_OK` on success
 */
esp_err_t hts221_is_data_ready(hts221_t *dev, bool *ready);

/**
 * @brief Start one shot measurement
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hts221_start_oneshot(hts221_t *dev);

/**
 * @brief Get temperature and relative humidity
 *
 * @param dev     Device descriptor
 * @param[out] t  Temperature, degrees Celsius
 * @param[out] rh Relative humidity, %
 * @return `ESP_OK` on success
 */
esp_err_t hts221_get_data(hts221_t *dev, float *t, float *rh);

/**
 * @brief Measure temperature and relative humidity in one shot mode
 *
 * @param dev     Device descriptor
 * @param[out] t  Temperature, degrees Celsius
 * @param[out] rh Relative humidity, %
 * @return `ESP_OK` on success
 */
esp_err_t hts221_measure(hts221_t *dev, float *t, float *rh);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __HTS221_H__

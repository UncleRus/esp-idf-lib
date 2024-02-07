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
 * @file si7021.h
 * @defgroup si7021 si7021
 * @{
 *
 * ESP-IDF driver for Si7013/Si7020/Si7021/HTU2xD/SHT2x and
 * compatible temperature and humidity sensors
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SI7021_H__
#define __SI7021_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SI7021_I2C_ADDR 0x40 //!< I2C address

#define SI7021_MAX_HEATER_CURRENT 0x0f //!< Maximum current of the heater for Si70xx

/**
 * Device model for Si70xx
 */
typedef enum {
    SI_MODEL_SI7013 = 0, //!< Si7013
    SI_MODEL_SI7020,     //!< Si7020
    SI_MODEL_SI7021,     //!< Si7021
    SI_MODEL_SAMPLE,     //!< Engineering sample
    SI_MODEL_UNKNOWN     //!< Unknown model
} si7021_device_id_t;

/**
 * Measurement resolution
 */
typedef enum {
    SI7021_RES_RH12_T14 = 0, //!< Relative humidity: 12 bits, temperature: 14 bits
    SI7021_RES_RH08_T12,     //!< Relative humidity: 8 bits, temperature: 12 bits
    SI7021_RES_RH10_T13,     //!< Relative humidity: 10 bits, temperature: 13 bits
    SI7021_RES_RH11_T11      //!< Relative humidity: 11 bits, temperature: 11 bits
} si7021_resolution_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param port      I2C port
 * @param sda_gpio  SDA GPIO pin
 * @param scl_gpio  SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t si7021_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t si7021_free_desc(i2c_dev_t *dev);

/**
 * @brief Reset device
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t si7021_reset(i2c_dev_t *dev);

/**
 * @brief Get heater state
 *
 * This function is supported by:
 *
 *  - SI7013
 *  - SI7020
 *  - SI7021
 *  - SHT2x
 *
 * @param dev      Device descriptor
 * @param[out] on  `true` if heater enabled
 * @return `ESP_OK` on success
 */
esp_err_t si7021_get_heater(i2c_dev_t *dev, bool *on);

/**
 * @brief Switch heater on/off
 *
 * This function is supported by:
 *
 *  - SI7013
 *  - SI7020
 *  - SI7021
 *  - SHT2x
 *
 * @param dev   Device descriptor
 * @param on    if `true`, heater will be enabled
 * @return `ESP_OK` on success
 */
esp_err_t si7021_set_heater(i2c_dev_t *dev, bool on);

/**
 * @brief Get heater current
 *
 * This function is supported by:
 *
 *  - SI7013
 *  - SI7020
 *  - SI7021
 *
 * @param dev         Device descriptor
 * @param[out] level  Heater current, see datasheet
 * @return `ESP_OK` on success
 */
esp_err_t si7021_get_heater_current(i2c_dev_t *dev, uint8_t *level);

/**
 * @brief Set heater current
 *
 * This function is supported by:
 *
 *  - SI7013
 *  - SI7020
 *  - SI7021
 *
 * @param dev       Device descriptor
 * @param level     Heater current, see datasheet
 * @return `ESP_OK` on success
 */
esp_err_t si7021_set_heater_current(i2c_dev_t *dev, uint8_t level);

/**
 * @brief Get measurement resolution
 *
 * @param dev       Device descriptor
 * @param[out] r    Resolution
 * @return `ESP_OK` on success
 */
esp_err_t si7021_get_resolution(i2c_dev_t *dev, si7021_resolution_t *r);

/**
 * @brief Set measurement resolution
 *
 * @param dev   Device descriptor
 * @param r     Resolution
 * @return `ESP_OK` on success
 */
esp_err_t si7021_set_resolution(i2c_dev_t *dev, si7021_resolution_t r);

/**
 * @brief Measure temperature
 *
 * @param dev     Device descriptor
 * @param[out] t  Temperature, deg. Celsius
 * @return `ESP_OK` on success
 */
esp_err_t si7021_measure_temperature(i2c_dev_t *dev, float *t);

/**
 * @brief Measure relative humidity
 *
 * @param dev       Device descriptor
 * @param[out] rh   Relative humidity, %
 * @return `ESP_OK` on success
 */
esp_err_t si7021_measure_humidity(i2c_dev_t *dev, float *rh);

/**
 * @brief Get serial number of device
 *
 * This function is supported by:
 *
 *  - SI7013
 *  - SI7020
 *  - SI7021
 *  - SHT2x
 *
 * @param dev          Device descriptor
 * @param[out] serial  Serial no.
 * @param sht2x_mode   `true` for SHT2x devices
 * @return `ESP_OK` on success
 */
esp_err_t si7021_get_serial(i2c_dev_t *dev, uint64_t *serial, bool sht2x_mode);

/**
 * @brief Get device model
 *
 * This function is supported by:
 *
 *  - SI7013
 *  - SI7020
 *  - SI7021
 *
 * @param dev       Device descriptor
 * @param[out] id   Device model
 * @return `ESP_OK` on success
 */
esp_err_t si7021_get_device_id(i2c_dev_t *dev, si7021_device_id_t *id);

/**
 * @brief Get device revision
 *
 * This function is supported by:
 *
 *  - SI7013
 *  - SI7020
 *  - SI7021
 *
 * @param dev       Device descriptor
 * @param[out] rev  Device revision
 * @return `ESP_OK` on success
 */
esp_err_t si7021_get_device_revision(i2c_dev_t *dev, uint8_t *rev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SI7021_H__ */

/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file hdc1000.h
 * @defgroup hdc1000 hdc1000
 * @{
 *
 * ESP-IDF driver for HDC1000 temperature and humidity sensor
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __HDC1000_H__
#define __HDC1000_H__

#include <i2cdev.h>
#include <stdbool.h>

#define HDC1000_I2C_ADDRESS_0 0x40 //!< I2C address when ADR1 = 0, ADR0 = 0
#define HDC1000_I2C_ADDRESS_1 0x41 //!< I2C address when ADR1 = 0, ADR0 = 1
#define HDC1000_I2C_ADDRESS_2 0x42 //!< I2C address when ADR1 = 1, ADR0 = 0
#define HDC1000_I2C_ADDRESS_3 0x43 //!< I2C address when ADR1 = 1, ADR0 = 1

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Temperature resolution
 */
typedef enum {
    HDC1000_T_RES_14 = 0, /**< 14 bits, default */
    HDC1000_T_RES_11,     /**< 11 bits */
} hdc1000_temperature_resolution_t;

/**
 * Humidity resolution
 */
typedef enum {
    HDC1000_H_RES_14 = 0, /**< 14 bits, default */
    HDC1000_H_RES_11,     /**< 11 bits */
    HDC1000_H_RES_8,      /**< 8 bits */
} hdc1000_humidity_resolution_t;

/**
 * Measurement mode
 */
typedef enum {
    HDC1000_MEASURE_TEMPERATURE = 0, /**< Temperature only */
    HDC1000_MEASURE_HUMIDITY,        /**< Humidity only */
    HDC1000_MEASURE_BOTH,            /**< Both temperature and humidity, default */
} hdc1000_measurement_mode_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    hdc1000_measurement_mode_t mode;
} hdc1000_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev      Device descriptor
 * @param addr     Device I2C address
 * @param port     I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_init_desc(hdc1000_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_free_desc(hdc1000_t *dev);

/**
 * @brief Init device
 *
 * Soft-reset device, set default measurement mode and resolutions
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_init(hdc1000_t *dev);

/**
 * @brief Read serial number of device
 *
 * @param dev Device descriptor
 * @param[out] serial Serial number
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_get_serial(hdc1000_t *dev, uint64_t *serial);

/**
 * @brief Read manufacturer ID of device
 *
 * @param dev Device descriptor
 * @param[out] id Manufacturer ID
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_get_manufacturer_id(hdc1000_t *dev, uint16_t *id);

/**
 * @brief Read device ID
 *
 * @param dev Device descriptor
 * @param[out] id Device ID
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_get_device_id(hdc1000_t *dev, uint16_t *id);

/**
 * @brief Read battery status
 *
 * @param dev Device descriptor
 * @param[out] undervolt true when battery voltage is lower than 2.8V
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_get_battery_status(hdc1000_t *dev, bool *undervolt);

/**
 * @brief Get heater status
 *
 * @param dev Device descriptor
 * @param[out] on true when heater is enabled
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_get_heater(hdc1000_t *dev, bool *on);

/**
 * @brief Switch heater on/off
 *
 * @param dev Device descriptor
 * @param on true to enable heater
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_set_heater(hdc1000_t *dev, bool on);

/**
 * @brief Set measurement mode
 *
 * @param dev Device descriptor
 * @param mode Measurement mode
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_set_measurement_mode(hdc1000_t *dev, hdc1000_measurement_mode_t mode);

/**
 * @brief Get measurement resolutions
 *
 * @param dev Device descriptor
 * @param[out] tres Temperature measurement resultion
 * @param[out] hres Humidity measurement resultion
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_get_resolution(hdc1000_t *dev, hdc1000_temperature_resolution_t *tres, hdc1000_humidity_resolution_t *hres);

/**
 * @brief Set measurement resolutions
 *
 * @param dev Device descriptor
 * @param tres Temperature measurement resultion
 * @param hres Humidity measurement resultion
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_set_resolution(hdc1000_t *dev, hdc1000_temperature_resolution_t tres, hdc1000_humidity_resolution_t hres);

/**
 * @brief Trigger measurement
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_trigger_measurement(hdc1000_t *dev);

/**
 * @brief Read output data
 *
 * @param dev Device descriptor
 * @param[out] t Temperature, degrees Celsius (nullable)
 * @param[out] rh Relative humidity, % (nullable)
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_get_data(hdc1000_t *dev, float *t, float *rh);

/**
 * @brief Measure
 *
 * Trigger measurement, wait and read output data
 *
 * @param dev Device descriptor
 * @param[out] t Temperature, degrees Celsius (nullable)
 * @param[out] rh Relative humidity, % (nullable)
 * @return `ESP_OK` on success
 */
esp_err_t hdc1000_measure(hdc1000_t *dev, float *t, float *rh);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __HDC1000_H__ */

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
 * @file sts21.h
 * @defgroup sts21 sts21
 * @{
 *
 * ESP-IDF driver for humidty/temperature sensors STS2110/STS2115/STS2120
 *
 * Copyright (c) 2022 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __sts21_H__
#define __sts21_H__

#include <i2cdev.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Measurement resolutions
 */
typedef enum {
    STS21_RESOLUTION_14 = 0, /**< 14 bits, <= 85 ms, default */
    STS21_RESOLUTION_13,     /**< 13 bits, <= 43 ms */
    STS21_RESOLUTION_12,     /**< 12 bits, <= 22 ms */
    STS21_RESOLUTION_11,     /**< 11 bits, <= 11 ms */
} sts21_resolution_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    sts21_resolution_t resolution;
} sts21_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev      Device descriptor
 * @param port     I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t sts21_init_desc(sts21_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sts21_free_desc(sts21_t *dev);

/**
 * @brief Init device
 *
 * Perform soft-reset, set resolution to 14 bits, switch off the heater.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sts21_init(sts21_t *dev);

/**
 * @brief Get measurement resolution
 *
 * @param dev      Device descriptor
 * @param[out] res Measurement resolution
 * @return `ESP_OK` on success
 */
esp_err_t sts21_get_resolution(sts21_t *dev, sts21_resolution_t *res);

/**
 * @brief Set measurement resolution
 *
 * @param dev Device descriptor
 * @param res Measurement resolution
 * @return `ESP_OK` on success
 */
esp_err_t sts21_set_resolution(sts21_t *dev, sts21_resolution_t res);

/**
 * @brief Get heater state
 *
 * @param dev     Device descriptor
 * @param[out] on true when heater is on
 * @return `ESP_OK` on success
 */
esp_err_t sts21_get_heater_state(sts21_t *dev, bool *on);

/**
 * @brief Switch heater on/off
 *
 * @param dev Device descriptor
 * @param on  true to switch heater on
 * @return `ESP_OK` on success
 */
esp_err_t sts21_set_heater_state(sts21_t *dev, bool on);

/**
 * @brief Get power state
 *
 * @param dev        Device descriptor
 * @param[out] alert true when power voltage < 2.25V
 * @return `ESP_OK` on success
 */
esp_err_t sts21_get_power_alert(sts21_t *dev, bool *alert);

/**
 * @brief Get measurement status
 *
 * @param dev       Device descriptor
 * @param[out] busy true when device performs measurement
 * @return `ESP_OK` on success
 */
esp_err_t sts21_is_busy(sts21_t *dev, bool *busy);

/**
 * @brief Trigger measurement
 *
 * Before reading the measurement result, you must wait a time depending
 * on the resolution or use ::sts21_is_busy() function
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sts21_trigger_measurement(sts21_t *dev);

/**
 * @brief Read measurement result
 *
 * @param dev    Device descriptor
 * @param[out] t Temperature in degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t sts21_read_temperature(sts21_t *dev, float *t);

/**
 * @brief Measure temperature and read result
 *
 * The function combines function ::sts21_trigger_measurement() and function
 * ::sts21_read_temperature() to get the measurement results.
 *
 * @param dev    Device descriptor
 * @param[out] t Temperature in degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t sts21_measure(sts21_t *dev, float *t);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __sts21_H__ */

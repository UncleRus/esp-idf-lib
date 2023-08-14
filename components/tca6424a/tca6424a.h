/*
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file tca6424a.h
 * @defgroup tca6424a tca6424a
 * @{
 *
 * ESP-IDF driver for TCA6424A low-voltage 24-bit I2C I/O expander
 *
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __TCA6424A_H__
#define __TCA6424A_H__

#include <stddef.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TCA6424A_I2C_ADDRESS_GND 0x22
#define TCA6424A_I2C_ADDRESS_VCC 0x23

/**
 * @brief Initialize device descriptor
 *
 * Default SCL frequency is 400kHz
 *
 * @param dev Pointer to I2C device descriptor
 * @param addr I2C address (`0b0100<A2><A1><A0>`)
 * @param port I2C port number
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_free_desc(i2c_dev_t *dev);

/**
 * @brief Get GPIO pins mode
 *
 * 0 - output, 1 - input for lower 24 bits in `val`
 *
 * @param dev Pointer to device descriptor
 * @param[out] mode Buffer to store mode, 0 bit for P00 .. 23 bit for P27
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_port_get_mode(i2c_dev_t *dev, uint32_t *mode);

/**
 * @brief Set GPIO pins mode
 *
 * 0 - output, 1 - input for lower 24 bits in `val`
 *
 * @param dev Pointer to device descriptor
 * @param mode Mode, 0 bit for P00 .. 23 bit for P27
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_port_set_mode(i2c_dev_t *dev, uint32_t mode);

/**
 * @brief Get GPIO pins polarity inversion
 *
 * 0 - no inversion, 1 - invert polarity for lower 24 bits in `val`
 *
 * @param dev Pointer to device descriptor
 * @param[out] polarity Polarity, 0 bit for P00 .. 23 bit for P27
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_port_get_polarity_inversion(i2c_dev_t *dev, uint32_t *polarity);

/**
 * @brief Set GPIO pins polarity inversion
 *
 * 0 - no inversion, 1 - invert polarity for lower 24 bits in `val`
 *
 * @param dev Pointer to device descriptor
 * @param polarity Polarity, 0 bit for P00 .. 23 bit for P27
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_port_set_polarity_inversion(i2c_dev_t *dev, uint32_t polarity);

/**
 * @brief Read GPIO port value
 *
 * @param dev Pointer to I2C device descriptor
 * @param val 24-bit GPIO port value, 0 bit for P00 .. 23 bit for P27
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_port_read(i2c_dev_t *dev, uint32_t *val);

/**
 * @brief Write value to GPIO port
 *
 * @param dev Pointer to I2C device descriptor
 * @param val GPIO port value, 0 bit for P00 .. 23 bit for P27
 * @return ESP_OK on success
 */
esp_err_t tca6424a_port_write(i2c_dev_t *dev, uint32_t val);

/**
 * @brief Read GPIO pin level
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 bit for P00 .. 23 bit for P27
 * @param[out] val `true` if pin currently in high state
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t *val);

/**
 * @brief Set GPIO pin level
 *
 * Pin must be set up as output
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 bit for P00 .. 23 bit for P27
 * @param[out] val `true` if pin currently in high state
 * @return `ESP_OK` on success
 */
esp_err_t tca6424a_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __TCA6424A_H__ */

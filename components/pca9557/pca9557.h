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
 * @file pca9557.h
 * @defgroup pca9557 pca9557
 * @{
 *
 * ESP-IDF driver for PCA9537/PCA9557/TCA9534 remote 4/8-bit I/O expanders for I2C-bus
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __PCA9557_H__
#define __PCA9557_H__

#include <stddef.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9537_I2C_ADDR      0x49 ///< I2C address for PCA9537
#define PCA9557_I2C_ADDR_BASE 0x18 ///< Base I2C address for PCA9557
#define TCA9534_I2C_ADDR_BASE 0x20 ///< Base I2C address for TCA9534

/**
 * Pin modes (directions)
 */
typedef enum {
    PCA9557_MODE_OUTPUT = 0,
    PCA9557_MODE_INPUT,
} pca9557_mode_t;

/**
 * @brief Initialize device descriptor
 *
 * Default SCL frequency is 400kHz
 *
 * @param dev       Pointer to I2C device descriptor
 * @param port      I2C port number
 * @param addr      I2C address (`0b0011<A2><A1><A0>` for PCA9557/`PCA9537_I2C_ADDR` for PCA9537, `TCA9534_I2C_ADDR_BASE` for TCA9534)
 * @param sda_gpio  SDA GPIO
 * @param scl_gpio  SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev       Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_free_desc(i2c_dev_t *dev);

/**
 * @brief Get directions of I/O pins
 *
 * 0 - output, 1 - input for each bit in `mode`
 *
 * @param dev       Pointer to device descriptor
 * @param[out] mode I/O directions
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_port_get_mode(i2c_dev_t *dev, uint8_t *mode);

/**
 * @brief Set directions of I/O pins
 *
 * 0 - output, 1 - input for each bit in `mode`
 *
 * @param dev       Pointer to device descriptor
 * @param mode      I/O directions
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_port_set_mode(i2c_dev_t *dev, uint8_t mode);

/**
 * @brief Get input polarity settings
 *
 * 0 - normal input polarity, 1 - inverted for each bit in `pol`
 *
 * @param dev       Pointer to device descriptor
 * @param[out] pol  Input polarity settings
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_port_get_polarity(i2c_dev_t *dev, uint8_t *pol);

/**
 * @brief Set input polarity settings
 *
 * 0 - normal input polarity, 1 - inverted for each bit in `pol`
 *
 * @param dev       Pointer to device descriptor
 * @param pol       Input polarity settings
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_port_set_polarity(i2c_dev_t *dev, uint8_t pol);

/**
 * @brief Read I/O port value
 *
 * @param dev       Pointer to I2C device descriptor
 * @param[out] val  8-bit GPIO port value for PCA9557 or 4-bit port value for PCA9537
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_port_read(i2c_dev_t *dev, uint8_t *val);

/**
 * @brief Write value to I/O port
 *
 * @param dev     Pointer to I2C device descriptor
 * @param val     8-bit GPIO port value for PCA9557 or 4-bit port value for PCA9537
 * @return ESP_OK on success
 */
esp_err_t pca9557_port_write(i2c_dev_t *dev, uint8_t val);

/**
 * @brief Read I/O pin mode
 *
 * @param dev       Pointer to device descriptor
 * @param pin       Pin number, 0..7 for PCA9557, 0..3 for PC9537
 * @param[out] mode Pin mode
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_get_mode(i2c_dev_t *dev, uint8_t pin, pca9557_mode_t *mode);

/**
 * @brief Set I/O pin mode
 *
 * @param dev      Pointer to device descriptor
 * @param pin      Pin number, 0..7 for PCA9557, 0..3 for PC9537
 * @param mode     Pin mode
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_set_mode(i2c_dev_t *dev, uint8_t pin, pca9557_mode_t mode);

/**
 * @brief Read I/O pin level
 *
 * @param dev      Pointer to device descriptor
 * @param pin      Pin number, 0..7 for PCA9557, 0..3 for PC9537
 * @param[out] val 1 if pin currently in high state, 0 otherwise
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t *val);

/**
 * @brief Set I/O pin level
 *
 * Pin must be set up as output
 *
 * @param dev      Pointer to device descriptor
 * @param pin      Pin number, 0..7 for PCA9557, 0..3 for PC9537
 * @param val      Pin level. 1 - high, 0 - low
 * @return `ESP_OK` on success
 */
esp_err_t pca9557_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PCA9557_H__ */

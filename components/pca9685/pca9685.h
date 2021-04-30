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
 * @file pca9685.h
 * @defgroup pca9685 pca9685
 * @{
 *
 * ESP-IDF driver for 16-channel, 12-bit PWM PCA9685
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __PCA9685_H__
#define __PCA9685_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9685_ADDR_BASE 0x40 //!< Base I2C device address

#define PCA9685_MAX_PWM_VALUE 4096

/**
 * PWM channel
 */
typedef enum
{
    PCA9685_CHANNEL_0 = 0,//!< PCA9685_CHANNEL_0
    PCA9685_CHANNEL_1,    //!< PCA9685_CHANNEL_1
    PCA9685_CHANNEL_2,    //!< PCA9685_CHANNEL_2
    PCA9685_CHANNEL_3,    //!< PCA9685_CHANNEL_3
    PCA9685_CHANNEL_4,    //!< PCA9685_CHANNEL_4
    PCA9685_CHANNEL_5,    //!< PCA9685_CHANNEL_5
    PCA9685_CHANNEL_6,    //!< PCA9685_CHANNEL_6
    PCA9685_CHANNEL_7,    //!< PCA9685_CHANNEL_7
    PCA9685_CHANNEL_8,    //!< PCA9685_CHANNEL_8
    PCA9685_CHANNEL_9,    //!< PCA9685_CHANNEL_9
    PCA9685_CHANNEL_10,   //!< PCA9685_CHANNEL_10
    PCA9685_CHANNEL_11,   //!< PCA9685_CHANNEL_11
    PCA9685_CHANNEL_12,   //!< PCA9685_CHANNEL_12
    PCA9685_CHANNEL_13,   //!< PCA9685_CHANNEL_13
    PCA9685_CHANNEL_14,   //!< PCA9685_CHANNEL_14
    PCA9685_CHANNEL_15,   //!< PCA9685_CHANNEL_15
    PCA9685_CHANNEL_ALL   //!< All channels
} pca9685_channel_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Pointer to I2C device descriptor
 * @param addr PCA9685 address
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_free_desc(i2c_dev_t *dev);

/**
 * @brief Init device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_init(i2c_dev_t *dev);

/**
 * @brief Setup device subaddress
 *
 * See section 7.3.6 if the datasheet
 *
 * @param dev Device descriptor
 * @param num Subaddress number, 0..2
 * @param subaddr Subaddress, 7 bit
 * @param enable True to enable subaddress, false to disable
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_set_subaddr(i2c_dev_t *dev, uint8_t num, uint8_t subaddr, bool enable);

/**
 * @brief Restart device
 *
 * See section 7.3.1.1 of the datasheet
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_restart(i2c_dev_t *dev);

/**
 * @brief Check if device is in sleep mode
 *
 * @param dev Device descriptor
 * @param[out] sleeping True if device is sleeping
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_is_sleeping(i2c_dev_t *dev, bool *sleeping);

/**
 * @brief Switch device to low-power mode or wake it up
 *
 * @param dev Device descriptor
 * @param sleep True for sleep mode, false for wake up
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_sleep(i2c_dev_t *dev, bool sleep);

/**
 * @brief Get logic inversion of the outputs
 *
 * @param dev Device descriptor
 * @param[out] inv True if outputs are inverted, false otherwise
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_is_output_inverted(i2c_dev_t *dev, bool *inv);

/**
 * @brief Logically invert outputs
 *
 * See section 7.7 of the datasheet
 *
 * @param dev Device descriptor
 * @param inverted True for inverted outputs
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_set_output_inverted(i2c_dev_t *dev, bool inverted);

/**
 * @brief Get outputs mode
 *
 * @param dev Device descriptor
 * @param[out] od True if outputs are in open drain mode
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_get_output_open_drain(i2c_dev_t *dev, bool *od);

/**
 * @brief Set outputs mode
 *
 * @param dev Device descriptor
 * @param od True to set open drain mode, false to normal mode
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_set_output_open_drain(i2c_dev_t *dev, bool od);

/**
 * @brief Get PWM frequency prescaler
 *
 * @param dev Device descriptor
 * @param[out] prescaler Frequency prescaler
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_get_prescaler(i2c_dev_t *dev, uint8_t *prescaler);

/**
 * @brief Set PWM frequency prescaler
 *
 * @param dev Device descriptor
 * @param prescaler Prescaler value
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_set_prescaler(i2c_dev_t *dev, uint8_t prescaler);

/**
 * @brief Get PWM frequency
 *
 * @param dev Device descriptor
 * @param[out] freq PWM frequency, Hz
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_get_pwm_frequency(i2c_dev_t *dev, uint16_t *freq);

/**
 * @brief Set PWM frequency
 *
 * @param dev Device descriptor
 * @param freq PWM frequency, Hz
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_set_pwm_frequency(i2c_dev_t *dev, uint16_t freq);

/**
 * @brief Set PWM value on output channel
 *
 * @param dev Device descriptor
 * @param channel Channel number, 0..15 or >15 for all channels
 * @param val PWM value, 0..4096
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_set_pwm_value(i2c_dev_t *dev, uint8_t channel, uint16_t val);

/**
 * @brief Set PWM values on multiple output channels
 *
 * @param dev Device descriptor
 * @param first_ch First channel, 0..15
 * @param channels Number of channels to update
 * @param values Array of the channel values, each 0..4096
 * @return `ESP_OK` on success
 */
esp_err_t pca9685_set_pwm_values(i2c_dev_t *dev, uint8_t first_ch, uint8_t channels,
        const uint16_t *values);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PCA9685_H__ */

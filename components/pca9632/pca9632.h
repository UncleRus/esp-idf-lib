/*
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
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
 * @file pca9632.h
 * @defgroup pca9632 pca9632
 * @{
 *
 * ESP-IDF Driver for PCA9632 4-channel PWM chip
 *
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __PCA9632_H__
#define __PCA9632_H__

#include <stddef.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9632_I2C_ADDR 0x62 ///< I2C address for PCA9632

// Auto-Increment options (page 10, table 6)
typedef enum {
    AI_DISABLED, //!< no Auto-Increment
    AI_ALL,      //!< Auto-Increment for all registers
    AI_IND,      //!< Auto-Increment for Individual brightness registers only
    AI_GBL,      //!< Auto-Increment for global control registers only
    AI_IND_GBL   //!< Auto-Increment for individual and global control registers only
} pca9632_autoincr_mode_t;

// Group control options (page 10, table 6)
typedef enum {
    GROUP_CONTROL_MODE_BLINKING, //!< group blink control
    GROUP_CONTROL_MODE_DIMMING   //!< group dimming control
} pca9632_gcm_t;

typedef enum {
    LED0 = 0, //!< LED0 PWM output
    LED1 = 1, //!< LED1 PWM output
    LED2 = 2, //!< LED2 PWM output
    LED3 = 3, //!< LED3 PWM output
} pca9632_led_t;

typedef enum {
    LDR_OFF = 0x00,     //!< LED driver x is off (default power-up state)
    LDR_ON = 0x01,      //!< LED driver x is fully on (individual brightness and group dimming/blinking not controlled)
    LDR_IND = 0x02,     //!< LED driver x individual brightness can be controlled through its PWMx register
    LDR_IND_GRP = 0x03, //!< LED driver x individual brightness and group dimming/blinking can be controlled through its PWMx register and the GRPPWM registers
} pca9632_ldr_t;

/**
 * @brief Initialize device descriptor
 *
 * Default SCL frequency is 400kHz
 *
 * @param dev       Pointer to I2C device descriptor
 * @param port      I2C port number
 * @param addr      I2C address
 * @param sda_gpio  SDA GPIO
 * @param scl_gpio  SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev       Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_free_desc(i2c_dev_t *dev);

esp_err_t pca9632_init(i2c_dev_t *dev);
esp_err_t pca9632_debug(i2c_dev_t *dev);

esp_err_t pca9632_set_autoincrement(i2c_dev_t *dev, pca9632_autoincr_mode_t ai);
esp_err_t pca9632_set_group_control_mode(i2c_dev_t *dev, pca9632_gcm_t mode);
esp_err_t pca9632_set_output_params(i2c_dev_t *dev, bool invert, bool outdrv);

esp_err_t pca9632_set_pwm(i2c_dev_t *dev, pca9632_led_t channel, uint8_t duty);
esp_err_t pca9632_set_pwm_all(i2c_dev_t *dev, uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3);

esp_err_t pca9632_set_grp_pwm(i2c_dev_t *dev, uint8_t val);
esp_err_t pca9632_set_grp_freq(i2c_dev_t *dev, uint8_t val);

esp_err_t pca9632_set_led_driver(i2c_dev_t *dev, pca9632_led_t channel, pca9632_ldr_t ldr);
esp_err_t pca9632_set_led_driver_all(i2c_dev_t *dev, pca9632_ldr_t ldr);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PCA9632_H__ */

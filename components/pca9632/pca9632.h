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
    AI_DISABLED,    //!< no Auto-Increment
    AI_ALL,         //!< Auto-Increment for all registers
    AI_INDIV,       //!< Auto-Increment for Individual brightness registers only
    AI_GLOBAL,      //!< Auto-Increment for global control registers only
    AI_INDIV_GLOBAL //!< Auto-Increment for individual and global control registers only
} pca9632_autoincr_mode_t;

// Group control options (page 10, table 6)
typedef enum {
    GROUP_CONTROL_MODE_BLINKING, //!< group blink control
    GROUP_CONTROL_MODE_DIMMING   //!< group dimming control
} pca9632_gcm_t;

typedef enum {
    OUTDRV_OPEN_DRAIN = 0, //!< open-drain output
    OUTDRV_TOTEM_POLE = 1  //!< totem-pole output
} pca9632_outdrv_t;

typedef enum {
    LED0 = 0, //!< LED0 PWM output
    LED1 = 1, //!< LED1 PWM output
    LED2 = 2, //!< LED2 PWM output
    LED3 = 3, //!< LED3 PWM output
} pca9632_led_t;

typedef enum {
    LDR_OFF = 0x00,       //!< LED driver x is off (default power-up state)
    LDR_ON = 0x01,        //!< LED driver x is fully on (individual brightness and group dimming/blinking not controlled)
    LDR_INDIV = 0x02,     //!< LED driver x individual brightness can be controlled through its PWMx register
    LDR_INDIV_GRP = 0x03, //!< LED driver x individual brightness and group dimming/blinking can be controlled through its PWMx register and the GRPPWM registers
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

/**
 * @brief Initialize device. Individual dimming mode by default
 *
 * @param dev       Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_init(i2c_dev_t *dev);

/**
 * @brief Set register address auto-increment mode, See 'pca9632_autoincr_mode_t' for details
 *
 * @param dev  Pointer to I2C device descriptor
 * @param ai   Auto-increment mode
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_autoincrement(i2c_dev_t *dev, pca9632_autoincr_mode_t ai);

/**
 * @brief Set group control mode. See 'pca9632_gcm_t' for details
 *
 * @param dev   Pointer to I2C device descriptor
 * @param mode  Group control mode
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_group_control_mode(i2c_dev_t *dev, pca9632_gcm_t mode);

/**
 * @brief Set up PWM outputs. See 'pca9632_outdrv_t' for details
 *
 * @param dev     Pointer to I2C device descriptor
 * @param invert  Use inverted logic on outputs
 * @param outdrv  Enable Totem-Pole output mode
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_output_params(i2c_dev_t *dev, bool invert, pca9632_outdrv_t outdrv);

/**
 * @brief Set PWM duty on selected channel
 *
 * @param dev      Pointer to I2C device descriptor
 * @param channel  PWM output channel
 * @param duty     PWM duty-cycle value
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_pwm(i2c_dev_t *dev, pca9632_led_t channel, uint8_t duty);

/**
 * @brief Set PWM duty on all channels
 *
 * @param dev   Pointer to I2C device descriptor
 * @param led0  PWM duty-cycle value on LED0 output
 * @param led1  PWM duty-cycle value on LED1 output
 * @param led2  PWM duty-cycle value on LED2 output
 * @param led3  PWM duty-cycle value on LED3 output
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_pwm_all(i2c_dev_t *dev, uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3);

/**
 * @brief Set GRPPWM register value
 *
 * @param dev  Pointer to I2C device descriptor
 * @param val  GRPPWM value
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_grp_pwm(i2c_dev_t *dev, uint8_t val);

/**
 * @brief Set GRPFREQ register value
 *
 * @param dev  Pointer to I2C device descriptor
 * @param val  GRPFREQ value
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_grp_freq(i2c_dev_t *dev, uint8_t val);

/**
 * @brief Set LED driver on selected channel. See 'pca9632_ldr_t' for details
 *
 * @param dev      Pointer to I2C device descriptor
 * @param channel  PWM output channel
 * @param ldr      LED driver type
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_led_driver(i2c_dev_t *dev, pca9632_led_t channel, pca9632_ldr_t ldr);

/**
 * @brief Set LED driver on all outputs. See 'pca9632_ldr_t' for details
 *
 * @param dev      Pointer to I2C device descriptor
 * @param ldr      LED driver type
 * @return `ESP_OK` on success
 */
esp_err_t pca9632_set_led_driver_all(i2c_dev_t *dev, pca9632_ldr_t ldr);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PCA9632_H__ */

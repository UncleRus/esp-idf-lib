/*
 * Copyright (c) 2016 Brian Schwind <https://github.com/bschwind>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file tsl2561.h
 * @defgroup tsl2561 tsl2561
 * @{
 *
 * ESP-IDF driver for TSL2561 light-to-digital converter
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Brian Schwind <https://github.com/bschwind>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __TSL2561_H__
#define __TSL2561_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSL2561_I2C_ADDR_GND   0x29
#define TSL2561_I2C_ADDR_FLOAT 0x39 //!< Default I2C address
#define TSL2561_I2C_ADDR_VCC   0x49

/**
 * Integration time
 */
typedef enum
{
    TSL2561_INTEGRATION_13MS = 0, //!< 13ms
    TSL2561_INTEGRATION_101MS,    //!< 101ms
    TSL2561_INTEGRATION_402MS     //!< 402ms, default
} tsl2561_integration_time_t;

/**
 * Gain
 */
typedef enum
{
    TSL2561_GAIN_1X = 0x00, //!< Default
    TSL2561_GAIN_16X = 0x10
} tsl2561_gain_t;

typedef enum
{
    TSL2561_PACKAGE_CS = 0,
    TSL2561_PACKAGE_T_FN_CL
} tsl2561_package_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    tsl2561_integration_time_t integration_time;
    tsl2561_gain_t gain;
    tsl2561_package_t package_type;
} tsl2561_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr I2C device address, `TSL2561_I2C_ADDR_...` const
 * @param port I2C port
 * @param sda_gpio SDA GPIO pin
 * @param scl_gpio SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_init_desc(tsl2561_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_free_desc(tsl2561_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_init(tsl2561_t *dev);

/**
 * @brief Set device integration time
 *
 * @param dev Device descriptor
 * @param integration_time Integration time
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_integration_time(tsl2561_t *dev, tsl2561_integration_time_t integration_time);

/**
 * @brief Set device gain
 *
 * @param dev Device descriptor
 * @param gain Gain
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_gain(tsl2561_t *dev, tsl2561_gain_t gain);

/**
 * @brief Read light intensity from device
 *
 * @param dev Device descriptor
 * @param[out] lux Light intensity, lux
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_read_lux(tsl2561_t *dev, uint32_t *lux);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __TSL2561_H__

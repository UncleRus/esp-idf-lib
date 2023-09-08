/*
 * Copyright (c) 2017 Pham Ngoc Thanh <pnt239@gmail.com>
 * Copyright (c) 2017 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file pcf8591.h
 * @defgroup pcf8591 pcf8591
 * @{
 *
 * ESP-IDF driver for 8-bit analog-to-digital conversion and
 * an 8-bit digital-to-analog conversion PCF8591
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2017 Pham Ngoc Thanh <pnt239@gmail.com>\n
 * Copyright (c) 2017 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __PCF8591_H__
#define __PCF8591_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCF8591_DEFAULT_ADDRESS 0x48

/**
 * Analog inputs configuration, see datasheet
 */
typedef enum {
    PCF8591_IC_4_SINGLES = 0,   //!< Four single-ended inputs
    PCF8591_IC_DIFF,            //!< Three differential inputs
    PCF8591_IC_2_SINGLES_DIFF,  //!< Two single-ended and differential mixed
    PCF8591_IC_2_DIFFS          //!< Two differential inputs
} pcf8591_input_conf_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr I2C device address
 * @param port I2C port number
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t pcf8591_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf8591_free_desc(i2c_dev_t *dev);

/**
 * @brief Read input value of an analog pin
 *
 * @param dev Device descriptor
 * @param conf Analog inputs configuration
 * @param channel Analog channel
 * @param[out] value Analog value
 * @return `ESP_OK` on success
 */
esp_err_t pcf8591_read(i2c_dev_t *dev, pcf8591_input_conf_t conf, uint8_t channel, uint8_t *value);

/**
 * @brief Write value to analog output
 *
 * @param dev Device descriptor
 * @param value DAC value
 * @return `ESP_OK` on success
 */
esp_err_t pcf8591_write(i2c_dev_t *dev, uint8_t value);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PCF8591_H__ */

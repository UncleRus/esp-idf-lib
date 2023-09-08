/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file tca9548.h
 * @defgroup tca9548 tca9548
 * @{
 *
 * ESP-IDF driver for low-voltage 8-channel I2C switch TCA9548/PCA9548
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __TCA9548_H__
#define __TCA9548_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TCA9548_ADDR_0 0x70
#define TCA9548_ADDR_1 0x71
#define TCA9548_ADDR_2 0x72
#define TCA9548_ADDR_3 0x73
#define TCA9548_ADDR_4 0x74
#define TCA9548_ADDR_5 0x75
#define TCA9548_ADDR_6 0x76
#define TCA9548_ADDR_7 0x77

#ifndef BV
#define BV(x) (1 << (x))
#endif

#define TCA9548_CHANNEL0 BV(0)
#define TCA9548_CHANNEL1 BV(1)
#define TCA9548_CHANNEL2 BV(2)
#define TCA9548_CHANNEL3 BV(3)
#define TCA9548_CHANNEL4 BV(4)
#define TCA9548_CHANNEL5 BV(5)
#define TCA9548_CHANNEL6 BV(6)
#define TCA9548_CHANNEL7 BV(7)

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param addr Device address
 * @param sda_gpio SDA GPIO pin
 * @param scl_gpio SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t tca9548_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tca9548_free_desc(i2c_dev_t *dev);

/**
 * @brief Switch channels
 *
 * @param dev Device descriptor
 * @param channels Channel flags, combination of TCA9548_CHANNELn
 * @return `ESP_OK` on success
 */
esp_err_t tca9548_set_channels(i2c_dev_t *dev, uint8_t channels);

/**
 * @brief Read current channels configuration
 *
 * @param dev Device descriptor
 * @param[out] channels Channel flags, combination of TCA9548_CHANNELn
 * @return `ESP_OK` on success
 */
esp_err_t tca9548_get_channels(i2c_dev_t *dev, uint8_t *channels);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __TCA9548_H__ */

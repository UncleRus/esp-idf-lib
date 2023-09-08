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
 * @file tsys01.h
 * @defgroup tsys01 tsys01
 * @{
 *
 * ESP-IDF driver for digital temperature sensor TSYS01
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __TSYS01_H__
#define __TSYS01_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSYS01_I2C_ADDR1 0x76
#define TSYS01_I2C_ADDR2 0x77

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev; //!< I2C device descriptor
    uint16_t cal[8];   //!< Calibration values
    uint32_t serial;   //!< Serial number
} tsys01_t;

/**
 * @brief Initialize device descriptor.
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_init_desc(tsys01_t *dev, uint8_t addr, i2c_port_t port,
                           gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_free_desc(tsys01_t *dev);

/**
 * @brief Initialize device.
 *
 * Reads sensor configuration.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_init(tsys01_t *dev);

/**
 * @brief Reset sensor.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_reset(tsys01_t *dev);

/**
 * @brief Start temperature conversion.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_start(tsys01_t *dev);

/**
 * @brief Read converted temperature from sensor.
 *
 * @param dev Device descriptor
 * @param[out] raw Raw ADC value, NULL-able
 * @param[out] t Temperature, degrees Celsius, NULL-able
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_get_temp(tsys01_t *dev, uint32_t *raw, float *t);

/**
 * @brief Perform temperature conversion
 *
 * This function starts temperature conversion,
 * waits 10 ms and reads result.
 *
 * @param dev Device descriptor
 * @param[out] t Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_measure(tsys01_t *dev, float *t);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __TSYS01_H__ */

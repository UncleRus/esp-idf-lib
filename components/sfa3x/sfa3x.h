/*
 * Copyright (c) 2021, Sensirion AG
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
 * @file sfa3x.h
 * @defgroup sfa3x sfa3x
 * @{
 *
 * ESP-IDF driver for SFA30 formaldehyde detection module
 *
 * Ported from https://github.com/Sensirion/embedded-sfa3x
 *
 * Copyright (c) 2021, Sensirion AG
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __sfa3x_H__
#define __sfa3x_H__

#include <i2cdev.h>
#include <esp_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SFA3X_I2C_ADDR 0x5d

/**
 * @brief Initialize device descriptor.
 *
 * @param dev      Device descriptor
 * @param port     I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return         `ESP_OK` on success
 */
esp_err_t sfa3x_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t sfa3x_free_desc(i2c_dev_t *dev);

/**
 * @brief Reset sensor.
 *
 * This function brings the sensor into the same state as after power-up.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t sfa3x_reset(i2c_dev_t *dev);

/**
 * @brief Start continuous measurement.
 *
 * After power up, the module is in Idle-Mode. Before any measurement values can be read,
 * the Measurement-Mode needs to be started using this command.
 *
 * @note This command is only available in idle mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t sfa3x_start_continuous_measurement(i2c_dev_t *dev);

/**
 * @brief Stop continuous measurement.
 *
 * Stop continuous measurement and return to idle mode for save energy.
 *
 * @note This command is only available in measurement mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t sfa3x_stop_continuous_measurement(i2c_dev_t *dev);

/**
 * @brief Read sensor output and convert.
 *
 * @note This command is only available in measurement mode.
 *
 * @param dev         Device descriptor
 * @param hcho        Formaldehyde concentration in ppb
 * @param humidity    Relative humidity in percent RH
 * @param temperature Temperature in degrees Celsius (°C)
 * @return            `ESP_OK` on success
 */
esp_err_t sfa3x_read_measurement(i2c_dev_t *dev, float *hcho, float *humidity, float *temperature);

/**
 * @brief Read device marking
 *
 * To identify individual sensors, the device marking string
 * (as printed on the sensor as 2D bar code) can be read with this function.
 *
 * @param dev     Device descriptor
 * @param marknig 32 bytes buffer to store marking as NULL-terminated string
 * @return        `ESP_OK` on success
 */
esp_err_t sfa3x_get_device_marknig(i2c_dev_t *dev, char *marknig);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __sfa3x_H__ */

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
 * @file ds1307.h
 * @defgroup ds1307 ds1307
 * @{
 *
 * ESP-IDF driver for DS1307 real-time clock
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __DS1307_H__
#define __DS1307_H__

#include <stdbool.h>
#include <time.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DS1307_ADDR 0x68 //!< I2C address

/**
 * Squarewave frequency
 */
typedef enum
{
    DS1307_1HZ = 0, //!< 1 Hz
    DS1307_4096HZ,  //!< 4096 Hz
    DS1307_8192HZ,  //!< 8192 Hz
    DS1307_32768HZ  //!< 32768 Hz
} ds1307_squarewave_freq_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_free_desc(i2c_dev_t *dev);

/**
 * @brief Start/stop clock
 *
 * @param dev Device descriptor
 * @param start Start clock if true
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_start(i2c_dev_t *dev, bool start);

/**
 * @brief Get current clock state
 *
 * @param dev Device descriptor
 * @param[out] running true if clock running
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_is_running(i2c_dev_t *dev, bool *running);

/**
 * @brief Get current time
 *
 * @param dev Device descriptor
 * @param[out] time Pointer to the time struct to fill
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_get_time(i2c_dev_t *dev, struct tm *time);

/**
 * @brief Set time to RTC
 *
 * @param dev Device descriptor
 * @param[in] time Pointer to the time struct
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_set_time(i2c_dev_t *dev, const struct tm *time);

/**
 * @brief Enable or disable square-wave oscillator output
 *
 * @param dev Device descriptor
 * @param enable Enable oscillator if true
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_enable_squarewave(i2c_dev_t *dev, bool enable);

/**
 * @brief Get square-wave oscillator output
 *
 * @param dev Device descriptor
 * @param[out] sqw_en true if square-wave oscillator enabled
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_is_squarewave_enabled(i2c_dev_t *dev, bool *sqw_en);

/**
 * @brief Set square-wave oscillator frequency
 *
 * @param dev Device descriptor
 * @param freq Frequency
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_set_squarewave_freq(i2c_dev_t *dev, ds1307_squarewave_freq_t freq);

/**
 * @brief Get current square-wave oscillator frequency
 *
 * @param dev Device descriptor
 * @param[out] sqw_freq Frequency
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_get_squarewave_freq(i2c_dev_t *dev, ds1307_squarewave_freq_t *sqw_freq);

/**
 * @brief Get current output level of the SQW/OUT pin
 *
 * @param dev Device descriptor
 * @param[out] out current output level of the SQW/OUT pin, true if high
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_get_output(i2c_dev_t *dev, bool *out);

/**
 * @brief Set output level of the SQW/OUT pin
 *
 * Set output level if square-wave output is disabled
 *
 * @param dev Device descriptor
 * @param value High level if true
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_set_output(i2c_dev_t *dev, bool value);

/**
 * @brief Read RAM contents into the buffer
 *
 * @param dev Device descriptor
 * @param offset Start byte, 0..55
 * @param[out] buf Buffer to store data
 * @param len Bytes to read, 1..56
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_read_ram(i2c_dev_t *dev, uint8_t offset, uint8_t *buf, uint8_t len);

/**
 * @brief Write buffer to RTC RAM
 *
 * @param dev Device descriptor
 * @param offset Start byte, 0..55
 * @param buf Buffer
 * @param len Bytes to write, 1..56
 * @return `ESP_OK` on success
 */
esp_err_t ds1307_write_ram(i2c_dev_t *dev, uint8_t offset, uint8_t *buf, uint8_t len);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __DS1307_H__ */

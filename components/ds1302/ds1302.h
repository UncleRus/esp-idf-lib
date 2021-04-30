/*
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2016 Pavel Merzlyakov <merzlyakovpavel@gmail.com>
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
 * @file ds1302.h
 * @defgroup ds1302 ds1302
 * @{
 *
 * ESP-IDF driver for DS1302 RTC
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>\n
 * Copyright (c) 2016 Pavel Merzlyakov <merzlyakovpavel@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __DS1302_H__
#define __DS1302_H__

#include <stdbool.h>
#include <driver/gpio.h>
#include <time.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DS1302_RAM_SIZE 31

/**
 * Device descriptor
 */
typedef struct
{
    gpio_num_t ce_pin;     //!< GPIO pin connected to CE
    gpio_num_t io_pin;     //!< GPIO pin connected to chip I/O
    gpio_num_t sclk_pin;   //!< GPIO pin connected to SCLK
    bool ch;               //!< true if clock is halted
} ds1302_t;

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_init(ds1302_t *dev);

/**
 * @brief Start/stop clock
 *
 * @param dev Device descriptor
 * @param start Start clock if true
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_start(ds1302_t *dev, bool start);

/**
 * @brief Get current clock state
 *
 * @param dev Device descriptor
 * @param[out] running true if clock running
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_is_running(ds1302_t *dev, bool *running);

/**
 * @brief Enable/disable write protection
 *
 * @param dev Device descriptor
 * @param wp Set RTC write-protected if true
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_set_write_protect(ds1302_t *dev, bool wp);

/**
 * @brief Get write protection status
 *
 * @param dev Device descriptor
 * @param[out] wp true if RTC write-protected
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_get_write_protect(ds1302_t *dev, bool *wp);

/**
 * @brief Get current time
 *
 * @param dev Device descriptor
 * @param[out] time Current time
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_get_time(ds1302_t *dev, struct tm *time);

/**
 * @brief Set time to RTC
 *
 * @param dev Device descriptor
 * @param time Time
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_set_time(ds1302_t *dev, const struct tm *time);

/**
 * @brief Read RAM contents into the buffer
 *
 * @param dev Device descriptor
 * @param offset Start byte, 0..55
 * @param[out] buf Buffer
 * @param len Bytes to read, 1..56
 *
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_read_sram(ds1302_t *dev, uint8_t offset, void *buf, uint8_t len);

/**
 * @brief Write buffer to RTC RAM
 *
 * @param dev Device descriptor
 * @param offset Start byte, 0..55
 * @param buf Buffer
 * @param len Bytes to write, 1..56
 *
 * @return `ESP_OK` on success
 */
esp_err_t ds1302_write_sram(ds1302_t *dev, uint8_t offset, void *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __DS1302_H__ */

/*
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
 * @file wiegand.h
 * @defgroup wiegand wiegand
 * @{
 *
 * ESP-IDF Wiegand protocol receiver
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __WIEGAND_H__
#define __WIEGAND_H__

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct wiegand_reader wiegand_reader_t;

typedef void (*wiegand_callback_t)(wiegand_reader_t *reader);

/**
 * Bit and byte order of data
 */
typedef enum {
    WIEGAND_MSB_FIRST = 0,
    WIEGAND_LSB_FIRST
} wiegand_order_t;

/**
 * Wiegand reader descriptor
 */
struct wiegand_reader
{
    gpio_num_t gpio_d0, gpio_d1;
    wiegand_callback_t callback;
    wiegand_order_t bit_order;
    wiegand_order_t byte_order;

    uint8_t *buf;
    size_t size;
    size_t bits;
    esp_timer_handle_t timer;
    bool start_parity;
    bool enabled;
};

/**
 * @brief Create and initialize reader instance.
 *
 * @param reader           Reader descriptor
 * @param gpio_d0          GPIO pin for D0
 * @param gpio_d1          GPIO pin for D0
 * @param internal_pullups Enable internal pull-up resistors for D0 and D1 GPIO
 * @param buf_size         Reader buffer size in bytes, must be large enough to
 *                         contain entire Wiegand key
 * @param callback         Callback function for processing received codes
 * @param bit_order        Bit order of data
 * @param byte_order       Byte order of data
 * @return `ESP_OK` on success
 */
esp_err_t wiegand_reader_init(wiegand_reader_t *reader, gpio_num_t gpio_d0, gpio_num_t gpio_d1,
        bool internal_pullups, size_t buf_size, wiegand_callback_t callback, wiegand_order_t bit_order,
        wiegand_order_t byte_order);

/**
 * @brief Disable reader
 *
 * While reader is disabled, it will not receive new data
 *
 * @param reader Reader descriptor
 * @return `ESP_OK` on success
 */
esp_err_t wiegand_reader_disable(wiegand_reader_t *reader);

/**
 * @brief Enable reader
 *
 * @param reader Reader descriptor
 * @return `ESP_OK` on success
 */
esp_err_t wiegand_reader_enable(wiegand_reader_t *reader);

/**
 * @brief Delete reader instance.
 *
 * @param reader Reader descriptor
 * @return `ESP_OK` on success
 */
esp_err_t wiegand_reader_done(wiegand_reader_t *reader);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __WIEGAND_H__ */

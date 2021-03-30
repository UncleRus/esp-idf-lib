/**
 * @file wiegand.h
 * @defgroup wiegand wiegand
 * @{
 *
 * ESP-IDF Wiegand protocol receiver
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
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
 * Wiegand reader descriptor
 */
struct wiegand_reader
{
    gpio_num_t gpio_d0, gpio_d1;
    wiegand_callback_t callback;

    uint8_t *buf;
    size_t size;
    size_t bits;
    esp_timer_handle_t timer;
    bool start_parity;
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
 * @return `ESP_OK` on success
 */
esp_err_t wiegand_reader_init(wiegand_reader_t *reader, gpio_num_t gpio_d0, gpio_num_t gpio_d1,
        bool internal_pullups, size_t buf_size, wiegand_callback_t callback);

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

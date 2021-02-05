/**
 * @file wiegand.h
 * @defgroup wiegand wiegand
 * @{
 *
 *
 * Copyright (C) 2020 Ruslan V. Uss <unclerus@gmail.com>
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

typedef struct
{
    uint16_t number;
    uint8_t facility;
    uint32_t raw;
} wiegand_fmt_26_hid_t;

typedef struct
{
    uint32_t cardholder;
    uint8_t facility;
    uint32_t raw;
} wiegand_fmt_31_hid_t;

typedef enum {
    WIEGAND_26_HID = 0,
    WIEGAND_31_HID,
    WIEGAND_32_HID,
    WIEGAND_33_HID,
} wiegand_fmt_t;

esp_err_t wiegand_reader_init(wiegand_reader_t *reader, gpio_num_t gpio_d0, gpio_num_t gpio_d1,
        bool internal_pullups, size_t buf_size, wiegand_callback_t callback);

esp_err_t wiegand_reader_done(wiegand_reader_t *reader);

esp_err_t wiegand_reader_decode(wiegand_reader_t *reader, wiegand_fmt_t fmt, void *res);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __WIEGAND_H__ */

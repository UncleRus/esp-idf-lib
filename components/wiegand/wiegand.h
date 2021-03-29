/**
 * @file wiegand.h
 * @defgroup wiegand wiegand
 * @{
 *
 * ESP-IDF component to communicate with Wiegand reader
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
 * Wiegand card
 */
typedef struct
{
    uint32_t issue_level;
    uint32_t facility;
    uint64_t number;
    uint32_t cardholder;
} wiegand_card_t;

/**
 * Wiegand code formats
 */
typedef enum {
    WIEGAND_H10301 = 0, /**< 26 bit HID (H10301) */
    WIEGAND_2804,       /**< 28 bit Wiegand */
    WIEGAND_ATS30,      /**< 30 bit ATS */
    WIEGAND_ADT31,      /**< 31 bit HID ADT */
    WIEGAND_KASTLE,     /**< 32 bit Kastle Systems */
    WIEGAND_D10202,     /**< 33 bit DSX-HID (D10202) F/C 17 ASSA Abloy IP Lockset */
    WIEGAND_H10306,     /**< 34 bit HID H10306, Honeywell/Northern N10002 */
    WIEGAND_C1000,      /**< 35 bit Corporate-1000 */
    WIEGAND_KS36,       /**< 36 bit Keyscan */
    WIEGAND_S12906,     /**< 36 bit HID Simplex S12906 */
    WIEGAND_SIEMENS,    /**< 36 bit Siemens */
    WIEGAND_H10302,     /**< 37 bit HID H10302 */
    WIEGAND_H10304,     /**< 37 bit HID H10304 */
    WIEGAND_P10001,     /**< 40 bit Honeywell P10001 */

    WIEGAND_FMT_MAX
} wiegand_format_t;

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

/**
 * @brief Decode internal reader buffer to Wiegand card
 *
 * @param reader     Reader descriptor
 * @param fmt        Target format
 * @param[out] card  Decoded result
 * @return `ESP_OK` on success
 */
esp_err_t wiegand_reader_decode(wiegand_reader_t *reader, wiegand_format_t fmt, wiegand_card_t *card);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __WIEGAND_H__ */

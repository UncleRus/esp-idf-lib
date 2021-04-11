/**
 * @file led_strip.h
 * @defgroup led_strip led_strip
 * @{
 *
 * RMT-based ESP-IDF driver for WS2812B/SK6812/APA106 LED strips
 *
 * Copyright (C) 2020 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __LED_STRIP_H__
#define __LED_STRIP_H__

#include <driver/gpio.h>
#include <esp_err.h>
#include <driver/rmt.h>
#include <color.h>

#ifdef __cplusplus
extern "C" {
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#define LED_STIRP_BRIGNTNESS
#endif

/**
 * LED type
 */
typedef enum
{
    LED_STRIP_WS2812 = 0,
    LED_STRIP_SK6812,
    LED_STRIP_APA106
} led_strip_type_t;

/**
 * @struct LED strip descriptor
 */
typedef struct
{
    led_strip_type_t type; ///< LED type
    bool is_rgbw;          ///< true for RGBW strips
#ifdef LED_STIRP_BRIGNTNESS
    uint8_t brightness;    ///< Brightness 0..255, call ::led_strip_flush() after change.
                           ///< Supported only for ESP-IDF version >= 4.4
#endif
    size_t length;         ///< Number of LEDs in strip
    gpio_num_t gpio;       ///< Data GPIO pin
    rmt_channel_t channel; ///< RMT channel
    uint8_t *buf;
} led_strip_t;

/**
 * @brief Setup library
 *
 * This method must be called before any other led_strip methods
 */
void led_strip_install();

/**
 * @brief Initialize LED strip and allocate buffer memory
 *
 * @param strip Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_init(led_strip_t *strip);

/**
 * @brief Deallocate buffer memory and release RMT channel
 *
 * @param strip Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_free(led_strip_t *strip);

/**
 * @brief Send strip buffer to LEDs
 *
 * @param strip Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_flush(led_strip_t *strip);

/**
 * @brief Check if associated RMT channel is busy
 *
 * @param strip Descriptor of LED strip
 * @return true if RMT peripherals is busy
 */
bool led_strip_busy(led_strip_t *strip);

/**
 * @brief Wait until RMT peripherals is free to send buffer to LEDs
 *
 * @param strip Descriptor of LED strip
 * @param timeout Timeout in RTOS ticks
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_wait(led_strip_t *strip, TickType_t timeout);

/**
 * @brief Set color of single LED in strip
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param num LED number, 0..strip length - 1
 * @param color RGB color
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_set_pixel(led_strip_t *strip, size_t num, rgb_t color);

/**
 * @brief Set colors of multiple LEDs
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param start First LED index, 0-based
 * @param len Number of LEDs
 * @param data Pointer to RGB data
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_set_pixels(led_strip_t *strip, size_t start, size_t len, rgb_t *data);

/**
 * @brief Set multiple LEDs to the one color
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param start First LED index, 0-based
 * @param len Number of LEDs
 * @param color RGB color
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_fill(led_strip_t *strip, size_t start, size_t len, rgb_t color);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_STRIP_H__ */

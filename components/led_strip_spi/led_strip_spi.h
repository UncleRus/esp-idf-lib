/*
 * MIT License
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *               2021 Tomoyuki Sakurai <y@rombik.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 *
 * @file led_strip_spi.h
 * @defgroup led_strip_spi led_strip_spi
 * @{
 *
 */
#ifndef __LED_STRIP_SPI_H__
#define __LED_STRIP_SPI_H__

#include <driver/gpio.h>
#include <esp_err.h>
#include <color.h>
#include <esp_idf_lib_helpers.h>

#if HELPER_TARGET_IS_ESP32
#include <driver/spi_master.h>
#include "led_strip_spi_esp32.h"
#define LED_STRIP_SPI_DEFAULT()   LED_STRIP_SPI_DEFAULT_ESP32() ///< an alias of `LED_STRIP_SPI_DEFAULT_ESP32()` or `LED_STRIP_SPI_DEFAULT_ESP8266()`
typedef led_strip_spi_esp32_t led_strip_spi_t;
#endif

#if HELPER_TARGET_IS_ESP8266
#include <driver/spi.h>
#include "led_strip_spi_esp8266.h"
#define LED_STRIP_SPI_DEFAULT()   LED_STRIP_SPI_DEFAULT_ESP8266()
typedef led_strip_spi_esp8266_t led_strip_spi_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * * add LED_STRIP_SPI_USING_$NAME to Kconfig
 * * define `LED_STRIP_SPI_BUFFER_SIZE(N_PIXEL)` that returns the required
 *   size of buffer for the $NAME.
 */

#if defined(CONFIG_LED_STRIP_SPI_USING_SK9822)
#include "led_strip_spi_sk9822.h"
#else
#error "unknown LED type"
#endif

/**
 * Maximum brightness value for a pixel.
 */
#define LED_STRIP_SPI_MAX_BRIGHTNESS (100)

/**
 * @brief Setup the driver
 *
 * This method must be called before any other led_strip_spi methods
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_install();

/**
 * @brief Initialize LED strip and allocate buffer memory
 *
 * @param strip Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_init(led_strip_spi_t*strip);

/**
 * @brief Free LED strip
 *
 * @param strip Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_free(led_strip_spi_t *strip);

/**
 * @brief Send strip buffer to LEDs
 * @param strip Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_flush(led_strip_spi_t*strip);

/**
 * @brief Set color of single LED in strip.
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_spi_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param num LED number, [0:strip.length - 1].
 * @param color RGB color
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_set_pixel(led_strip_spi_t *strip, const int num, const rgb_t color);

/**
 * @brief Set color of single LED in strip.
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_spi_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param num LED number, [0:strip.length - 1].
 * @param color RGB color
 * @param brightness Brightness of the LED, [0:100].
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_set_pixel_brightness(led_strip_spi_t *strip, const int num, const rgb_t color, const uint8_t brightness);

/**
 * @brief Set colors of multiple LEDs
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_spi_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param start First LED index, 0-based
 * @param len Number of LEDs
 * @param data Pointer to RGB data
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_set_pixels(led_strip_spi_t*strip, const int start, size_t len, const rgb_t data);

/**
 * @brief Set colors of multiple LEDs
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_spi_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param start First LED index, 0-based
 * @param len Number of LEDs
 * @param data Pointer to RGB data
 * @param brightness Brightness of the LED, [0:100].
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_set_pixels_brightness(led_strip_spi_t*strip, const int start, size_t len, const rgb_t data, const uint8_t brightness);

/**
 * @brief Set multiple LEDs to the one color.
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_spi_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param start First LED index, 0-based
 * @param len Number of LEDs
 * @param color RGB color
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_fill(led_strip_spi_t*strip, size_t start, size_t len, rgb_t color);

/**
 * @brief Set multiple LEDs to the one color.
 *
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_spi_flush() to send buffer to the LEDs.
 *
 * @param strip Descriptor of LED strip
 * @param start First LED index, 0-based
 * @param len Number of LEDs
 * @param color RGB color
 * @param brightness Brightness of the LEDs, [0:100].
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_fill_brightness(led_strip_spi_t*strip, size_t start, size_t len, rgb_t color, const uint8_t brightness);
#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_STRIP_SPI_H__ */

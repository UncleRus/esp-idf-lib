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
 * @file led_strip_spi_sk9822.h
 * @defgroup led_strip_spi_sk9822 led_strip_spi_sk9822
 * @{
 *
 * Functions and macros for SK9822 LED strips.
 *
 * SPI data frame consists of:
 *
 * - A start frame of 32 zero bits (<0x00> <0x00> <0x00> <0x00>).
 * - 32 bit LED frames for each LED in the string (<0xE0+brightness> &lt;blue&gt;
 *   &lt;green&gt; &lt;red&gt;).
 * - A SK9822 reset frame of 32 zero bits (<0x00> <0x00> <0x00> <0x00>).
 * - An end frame consisting of at least (n/2) bits of 1, where n is the
 *   number of LEDs in the string.
 *
 * For the details, see
 * [SK9822 – a clone of the APA102?](https://cpldcpu.wordpress.com/2016/12/13/sk9822-a-clone-of-the-apa102/)
 * and
 * [Understanding the APA102 “Superled”](https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/).
 *
 */
#if !defined(__LED_STRIP_SPI_SK9822_H__)
#define __LED_STRIP_SPI_SK9822_H__

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_STRIP_SPI_FRAME_SK9822_START_SIZE  (4)      ///< The size in bytes of start frame.
#define LED_STRIP_SPI_FRAME_SK9822_LED_SIZE    (4)      ///< The size in bytes of each LED frame.
#define LED_STRIP_SPI_FRAME_SK9822_LEDS_SIZE(N_PIXEL) (LED_STRIP_SPI_FRAME_SK9822_LED_SIZE * N_PIXEL) ///< Total size in bytes of all LED frames in a strip. `N_PIXEL` is the number of pixels in the strip.
#define LED_STRIP_SPI_FRAME_SK9822_RESET_SIZE  (4)      ///< The size in bytes of reset frame.
#define LED_STRIP_SPI_FRAME_SK9822_END_SIZE(N_PIXEL) ((N_PIXEL / 16) + 1) ///< The size in bytes of the last frame. `N_PIXEL` is the number of pixels in the strip.

#define LED_STRIP_SPI_FRAME_SK9822_LED_MSB3    (0xE0)   ///< A magic number of [31:29] in LED frames. The bits must be 1 (APA102, SK9822)

#define LED_STRIP_SPI_FRAME_SK9822_LED_BRIGHTNESS_BITS (5) ///< Number of bits used to describe the brightness of the LED

#define LED_STRIP_SPI_BUFFER_SIZE(N_PIXEL) (\
        LED_STRIP_SPI_FRAME_SK9822_START_SIZE + \
        LED_STRIP_SPI_FRAME_SK9822_LEDS_SIZE(N_PIXEL)  + \
        LED_STRIP_SPI_FRAME_SK9822_RESET_SIZE + \
        LED_STRIP_SPI_FRAME_SK9822_END_SIZE(N_PIXEL)) ///< A macro to caliculate required size of buffer. `N_PIXEL` is the number of pixels in the strip.

/**
 * @brief Initialize the buffer of SK9822 strip.
 * @param[in] strip LED strip descriptor to initialize
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_sk9822_buf_init(led_strip_spi_t *strip);

/**
 * @brief Set color of a pixel of SK9822 strip.
 * @param[in] strip LED strip descriptor.
 * @param[in] num Index of the LED pixel (zero-based).
 * @param[in] color The color to set.
 * @param[in] brightness The brightness to set, [0:100].
 * @return `ESP_OK` on success.
 */
esp_err_t led_strip_spi_set_pixel_sk9822(led_strip_spi_t *strip, size_t num, rgb_t color, uint8_t brightness);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif

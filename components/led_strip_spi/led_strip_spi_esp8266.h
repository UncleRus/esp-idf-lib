/*
 * MIT License
 *
 * Copyright (c) 2021 Tomoyuki Sakurai <y@rombik.org>
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

#if !defined(__LED_STRIP_SPI_ESP8266__H__)
#define __LED_STRIP_SPI_ESP8266__H__

/**
 * @file led_strip_spi_esp8266.h
 * @defgroup led_strip_spi_esp8266 led_strip_spi_esp8266
 *
 * @{
 */

#include <driver/spi.h>
#include "led_strip_spi_esp8266.h"

/**
 * @struct led_strip_spi_esp8266_t
 *
 * LED strip descriptor for ESP8266.
 */
typedef struct
{
    void *buf;              ///< Pointer to the buffer.
    size_t length;          ///< Number of pixels.
    spi_clk_div_t clk_div;  ///< Value of `clk_div`, such as `SPI_2MHz_DIV`. See available values in `${IDF_PATH}/components/esp8266/include/driver/spi.h`.
} led_strip_spi_esp8266_t;

/**
 * @brief A macro to initialize led_strip_spi_esp8266_t.
 *
 * `length`: 1 `clk_div`: SPI_2MHz_DIV
 */
#define LED_STRIP_SPI_DEFAULT_ESP8266() \
{ \
    .length = 1, \
    .clk_div = SPI_2MHz_DIV, \
}

/** @} */

#endif // __LED_STRIP_SPI_ESP8266__H__

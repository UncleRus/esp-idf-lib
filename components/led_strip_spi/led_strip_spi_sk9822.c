/*
 * MIT License
 *
 * Copyright (C) 2021 Tomoyuki Sakurai <y@rombik.org>
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

#include <esp_err.h>
#include "led_strip_spi.h"
#include "led_strip_spi_sk9822.h"

esp_err_t led_strip_spi_set_pixel_sk9822(led_strip_spi_t *strip, size_t num, rgb_t color)
{
    int index = (num + 1) * 4;
    ((uint8_t *)strip->buf)[index    ] = LED_STRIP_SPI_FRAME_SK9822_LED_MSB3 + 1; // XXX FIXME brightness control
    ((uint8_t *)strip->buf)[index + 1] = color.b;
    ((uint8_t *)strip->buf)[index + 2] = color.g;
    ((uint8_t *)strip->buf)[index + 3] = color.r;
    return ESP_OK;
}

esp_err_t led_strip_spi_sk9822_buf_init(led_strip_spi_t *strip)
{
    /* set mandatory bits in all LED frames */
    for (int i = 1; i <= strip->length; i++) {
        ((uint8_t *)strip->buf)[i * 4] = LED_STRIP_SPI_FRAME_SK9822_LED_MSB3;
    }
    return ESP_OK;
}

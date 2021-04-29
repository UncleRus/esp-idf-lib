/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 FastLED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
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
 * @file hsv.h
 * @defgroup hsv hsv
 * @{
 *
 * Functions for HSV colors
 *
 * Ported from FastLED
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __COLOR_HSV_H__
#define __COLOR_HSV_H__

#include <stdint.h>
#include <stdbool.h>
#include <lib8tion.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HUE_RED    0
#define HUE_ORANGE 32
#define HUE_YELLOW 64
#define HUE_GREEN  96
#define HUE_AQUA   128
#define HUE_BLUE   160
#define HUE_PURPLE 192
#define HUE_PINK   224

typedef enum
{
    COLOR_FORWARD_HUES = 0,
    COLOR_BACKWARD_HUES,
    COLOR_SHORTEST_HUES,
    COLOR_LONGEST_HUES
} color_gradient_direction_t;

/// HSV color representation
typedef struct
{
    union {
        uint8_t h;
        uint8_t hue;
    };
    union {
        uint8_t s;
        uint8_t sat;
        uint8_t saturation;
    };
    union {
        uint8_t v;
        uint8_t val;
        uint8_t value;
    };
} hsv_t;

/// This allows testing a HSV for zero-ness
static inline bool hsv_is_zero(hsv_t a)
{
    return !(a.h | a.s | a.v);
}

/// Create HSV color from values
static inline hsv_t hsv_from_values(uint8_t h, uint8_t s, uint8_t v)
{
    hsv_t res = {
        .h = h,
        .s = s,
        .v = v
    };
    return res;
}

/// Computes a new color blended some fraction of the way between two other
/// colors.
hsv_t blend(hsv_t existing, hsv_t overlay, fract8 amount, color_gradient_direction_t direction);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __COLOR_HSV_H__ */

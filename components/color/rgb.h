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
 * @file rgb.h
 * @defgroup rgb rgb
 * @{
 *
 * Functions for RGB colors
 *
 * Ported from FastLED
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __COLOR_RGB_H__
#define __COLOR_RGB_H__

#include <stdint.h>
#include <stdbool.h>
#include <lib8tion.h>

#ifdef __cplusplus
extern "C" {
#endif

/// RGB color representation
typedef struct
{
    union {
        uint8_t r;
        uint8_t red;
    };
    union {
        uint8_t g;
        uint8_t green;
    };
    union {
        uint8_t b;
        uint8_t blue;
    };
} rgb_t;

/// This allows testing a RGB for zero-ness
static inline bool rgb_is_zero(rgb_t a)
{
    return !(a.r | a.g | a.b);
}

/// Create rgb_t color from 24-bit color code 0x00RRGGBB
static inline rgb_t rgb_from_code(uint32_t color_code)
{
    rgb_t res = {
        .r = (uint8_t)((color_code >> 16) & 0xff),
        .g = (uint8_t)((color_code >> 8) & 0xff),
        .b = (uint8_t)(color_code & 0xff),
    };
    return res;
}

/// Create rgb_t color from values
static inline rgb_t rgb_from_values(uint8_t r, uint8_t g, uint8_t b)
{
    rgb_t res = {
        .r = r,
        .g = g,
        .b = b,
    };
    return res;
}

/// Convert RGB color to 24-bit color code 0x00RRGGBB
static inline uint32_t rgb_to_code(rgb_t color)
{
    return ((uint32_t)color.r << 16) | ((uint32_t)color.g << 8) | color.b;
}

/// Add a constant to each channel of RGB color,
/// saturating at 0xFF
static inline rgb_t rgb_add(rgb_t a, uint8_t val)
{
    rgb_t res = {
        .r = qadd8(a.r, val),
        .g = qadd8(a.g, val),
        .b = qadd8(a.b, val),
    };
    return res;
}

/// Subtract a constant from each channel of RGB color,
/// saturating at 0x00
static inline rgb_t rgb_sub(rgb_t a, uint8_t val)
{
    rgb_t res = {
        .r = qsub8(a.r, val),
        .g = qsub8(a.g, val),
        .b = qsub8(a.b, val),
    };
    return res;
}

/// Multiply each of the channels by a constant,
/// saturating each channel at 0xFF
static inline rgb_t rgb_mul(rgb_t a, uint8_t val)
{
    rgb_t res = {
        .r = qmul8(a.r, val),
        .g = qmul8(a.g, val),
        .b = qmul8(a.b, val),
    };
    return res;
}

/// Add one RGB to another, saturating at 0xFF for each channel
static inline rgb_t rgb_add_rgb(rgb_t a, rgb_t b)
{
    rgb_t res = {
        .r = qadd8(a.r, b.r),
        .g = qadd8(a.g, b.g),
        .b = qadd8(a.b, b.b),
    };
    return res;
}

/// Subtract one RGB from another, saturating at 0x00 for each channel
static inline rgb_t rgb_sub_rgb(rgb_t a, rgb_t b)
{
    rgb_t res = {
        .r = qsub8(a.r, b.r),
        .g = qsub8(a.g, b.g),
        .b = qsub8(a.b, b.b),
    };
    return res;
}

/// Scale down a RGB to N 256ths of it's current brightness, using
/// 'plain math' dimming rules, which means that if the low light levels
/// may dim all the way to 100% black.
static inline rgb_t rgb_scale(rgb_t a, uint8_t scaledown)
{
    rgb_t res = {
        .r = scale8(a.r, scaledown),
        .g = scale8(a.g, scaledown),
        .b = scale8(a.b, scaledown),
    };
    return res;
}

/// Scale down a RGB to N 256ths of it's current brightness, using
/// 'video' dimming rules, which means that unless the scale factor is ZERO
/// each channel is guaranteed NOT to dim down to zero.  If it's already
/// nonzero, it'll stay nonzero, even if that means the hue shifts a little
/// at low brightness levels.
static inline rgb_t rgb_scale_video(rgb_t a, uint8_t scaledown)
{
    rgb_t res = {
        .r = scale8_video(a.r, scaledown),
        .g = scale8_video(a.g, scaledown),
        .b = scale8_video(a.b, scaledown),
    };
    return res;
}

/// rgb_fade_light is a synonym for rgb_scale_video(..., 255 - fade_factor)
static inline rgb_t rgb_fade_light(rgb_t a, uint8_t fade_factor)
{
    return rgb_scale_video(a, ~fade_factor);
}

/// rgb_fade_light is a synonym for rgb_scale(..., 255 - fade_factor)
static inline rgb_t rgb_fade(rgb_t a, uint8_t fade_factor)
{
    return rgb_scale(a, ~fade_factor);
}

/// Invert each channel of RGB color
static inline rgb_t rgb_invert(rgb_t a)
{
    rgb_t res = {
        .r = (uint8_t) ~a.r,
        .g = (uint8_t) ~a.g,
        .b = (uint8_t) ~a.b,
    };
    return res;
}

/// Get the 'luma' of a RGB color - aka roughly how much light the RGB pixel
/// is putting out (from 0 to 255).
static inline uint8_t rgb_luma(rgb_t a)
{
    return scale8(a.r, 54) + scale8(a.g, 183) + scale8(a.b, 18);
}

/// Get the average of the R, G, and B values
static inline uint8_t rgb_average_light(rgb_t a)
{
    return scale8(a.r, 85) + scale8(a.g, 85) + scale8(a.b, 85);
}

/// Maximize the brightness of this RGB color
static inline rgb_t rgb_max_brightness(rgb_t a, uint8_t limit)
{
    uint8_t max = a.r;
    if (a.g > max) max = a.g;
    if (a.b > max) max = a.b;

    // stop div/0 when color is black
    if (!max) return a;

    uint16_t factor = ((uint16_t)(limit) * 256) / max;
    rgb_t res = {
        .r = (uint8_t)((a.r * factor) / 256),
        .g = (uint8_t)((a.g * factor) / 256),
        .b = (uint8_t)((a.b * factor) / 256),
    };
    return res;
}

/// Return a new RGB color after performing a linear interpolation between
/// colors a and b
static inline rgb_t rgb_lerp8(rgb_t a, rgb_t b, fract8 frac)
{
    rgb_t res = {
        .r = lerp8by8(a.r, b.r, frac),
        .g = lerp8by8(a.g, b.g, frac),
        .b = lerp8by8(a.b, b.b, frac),
    };
    return res;
}

/// Return a new RGB color after performing a linear interpolation between
/// colors a and b
static inline rgb_t rgb_lerp16(rgb_t a, rgb_t b, fract16 frac)
{
    rgb_t res = {
        .r = (uint8_t)lerp16by16(a.r, b.r, frac),
        .g = (uint8_t)lerp16by16(a.g, b.g, frac),
        .b = (uint8_t)lerp16by16(a.b, b.b, frac),
    };
    return res;
}

/// Computes a new color blended some fraction of the way between two other
/// colors.
static inline rgb_t rgb_blend(rgb_t existing, rgb_t overlay, fract8 amount)
{
    rgb_t res = {
        .r = blend8(existing.r, overlay.r, amount),
        .g = blend8(existing.g, overlay.g, amount),
        .b = blend8(existing.b, overlay.b, amount),
    };
    return res;
}

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __COLOR_RGB_H__ */

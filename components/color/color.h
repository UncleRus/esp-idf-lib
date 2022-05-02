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
 * @file color.h
 * @defgroup color color
 * @{
 *
 * Functions for RGB and HSV colors
 *
 * Ported from FastLED
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __COLOR_H__
#define __COLOR_H__

#include <stdint.h>

#include "rgb.h"
#include "hsv.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HUE_MAX_RAINBOW  255
#define HUE_MAX_SPECTRUM 255
#define HUE_MAX_RAW      191

////////////////////////////////////////////////////////////////////////////////
// Color conversion

/**
 * @brief Convert HSV to RGB using balanced rainbow
 *
 * Convert a hue, saturation, and value to RGB using a visually balanced
 * rainbow (vs a straight mathematical spectrum). This 'rainbow' yields
 * better yellow and orange than a straight 'spectrum'.
 *
 * @note here hue is 0-255, not just 0-191
 *
 * @param hsv   HSV color
 * @return      RGB color
 */
rgb_t hsv2rgb_rainbow(hsv_t hsv);

/**
 * @brief Convert HSV to RGB using mathematically straight spectrum
 *
 * Convert a hue, saturation, and value to RGB using a mathematically straight
 * spectrum (vs a visually balanced rainbow). This 'spectrum' will have more
 * green & blue than a 'rainbow', and less yellow and orange.
 *
 * @note here hue is 0-255, not just 0-191
 *
 * @param hsv   HSV color
 * @return      RGB color
 */
rgb_t hsv2rgb_spectrum(hsv_t hsv);

/**
 * @brief Convert HSV to RGB using spectrum
 *
 * Convert hue, saturation, and value to RGB. This 'spectrum' conversion will
 * be more green & blue than a real 'rainbow', and the hue is specified just in
 * the range 0-191.
 * Together, these result in a slightly faster conversion speed, at the expense
 * of color balance.
 *
 * @note Hue is 0-191 only! Saturation & value are 0-255 each.
 *
 * @param hsv   HSV color
 * @return      RGB color
 */
rgb_t hsv2rgb_raw(hsv_t hsv);

/**
 * @brief Recover approximate HSV values from RGB
 *
 * This function is a long-term work in process; expect results to change
 * slightly over time as this function is refined and improved.
 *
 * This function is most accurate when the input is an RGB color that came
 * from a fully-saturated HSV color to start with.
 *
 * This function is not nearly as fast as HSV-to-RGB. It is provided for
 * those situations when the need for this function cannot be avoided, or
 * when extremely high performance is not needed.
 *
 * Why is this 'only' an "approximation"? Not all RGB colors have HSV
 * equivalents! For example, there is no HSV value that will ever convert
 * to RGB(255,255,0) using the code provided in this library.
 * So if you try to convert RGB(255,255,0) 'back' to HSV, you'll necessarily
 * get only an approximation.
 * Emphasis has been placed on getting the 'hue' as close as usefully possible,
 * but even that's a bit of a challenge.
 * The 8-bit HSV and 8-bit RGB color spaces are not a "bijection".
 * Nevertheless, this function does a pretty good job, particularly at
 * recovering the 'hue' from fully saturated RGB colors that originally came
 * from HSV rainbow colors.
 * The more desaturated the original RGB color is, the rougher the approximation,
 * and the less accurate the results.
 *
 * @param rgb   RGB color
 * @return      Approximated HSV color
 */
hsv_t rgb2hsv_approximate(rgb_t rgb);

/**
 * @brief Approximates a 'black body radiation' spectrum for a given 'heat' level.
 *
 * This is useful for animations of 'fire'. Heat is specified as an arbitrary scale
 * from 0 (cool) to 255 (hot).
 * This is NOT a chromatically correct 'black body radiation' spectrum, but it's
 * surprisingly close, and it's fast and small.
 */
rgb_t rgb_heat_color(uint8_t temperature);

////////////////////////////////////////////////////////////////////////////////
// Fill functions

/**
 * Fill an array of HSV colors with a solid HSV color
 */
void hsv_fill_solid_hsv(hsv_t *target, hsv_t color, size_t num);

/**
 * Fill an array of RGB colors with a solid HSV color
 */
void rgb_fill_solid_hsv(rgb_t *target, hsv_t color, size_t num);

/**
 * Fill an array of RGB colors with a solid RGB color
 */
void rgb_fill_solid_rgb(rgb_t *target, rgb_t color, size_t num);

/**
 * @brief Fill an array of HSV colors with a smooth HSV gradient between two
 *        specified HSV colors.
 *
 * Since 'hue' is a value around a color wheel,
 * there are always two ways to sweep from one hue
 * to another.
 * This function lets you specify which way you want
 * the hue gradient to sweep around the color wheel:
 *
 *     FORWARD_HUES: hue always goes clockwise
 *     BACKWARD_HUES: hue always goes counter-clockwise
 *     SHORTEST_HUES: hue goes whichever way is shortest
 *     LONGEST_HUES: hue goes whichever way is longest
 *
 * The default is SHORTEST_HUES, as this is nearly
 * always what is wanted.
 */
void hsv_fill_gradient_hsv(hsv_t *target, size_t startpos, hsv_t startcolor, size_t endpos, hsv_t endcolor,
        color_gradient_direction_t direction);

static inline void hsv_fill_gradient2_hsv(hsv_t *target, size_t num, hsv_t c1, hsv_t c2,
        color_gradient_direction_t direction)
{
    hsv_fill_gradient_hsv(target, 0, c1, num - 1, c2, direction);
}

static inline void hsv_fill_gradient3_hsv(hsv_t *target, size_t num, hsv_t c1, hsv_t c2, hsv_t c3,
        color_gradient_direction_t direction)
{
    size_t half = num / 2;
    hsv_fill_gradient_hsv(target, 0,    c1, half,    c2, direction);
    hsv_fill_gradient_hsv(target, half, c2, num - 1, c3, direction);
}

static inline void hsv_fill_gradient4_hsv(hsv_t *target, size_t num, hsv_t c1, hsv_t c2, hsv_t c3, hsv_t c4,
        color_gradient_direction_t direction)
{
    size_t onethird = num / 3;
    size_t twothirds = num * 2 / 3;
    hsv_fill_gradient_hsv(target, 0,         c1, onethird,  c2, direction);
    hsv_fill_gradient_hsv(target, onethird,  c2, twothirds, c3, direction);
    hsv_fill_gradient_hsv(target, twothirds, c3, num - 1,   c4, direction);
}

/**
 * Same as ::hsv_fill_gradient_hsv(), but for array of RGB
 *
 * The gradient is computed in HSV space, and then HSV values are
 * converted to RGB as they're written into the RGB array.
 */
void rgb_fill_gradient_hsv(rgb_t *target, size_t startpos, hsv_t startcolor, size_t endpos, hsv_t endcolor,
        color_gradient_direction_t direction);

static inline void rgb_fill_gradient2_hsv(rgb_t *target, size_t num, hsv_t c1, hsv_t c2,
        color_gradient_direction_t direction)
{
    rgb_fill_gradient_hsv(target, 0, c1, num - 1, c2, direction);
}

static inline void rgb_fill_gradient3_hsv(rgb_t *target, size_t num, hsv_t c1, hsv_t c2, hsv_t c3,
        color_gradient_direction_t direction)
{
    size_t half = num / 2;
    rgb_fill_gradient_hsv(target, 0,    c1, half,    c2, direction);
    rgb_fill_gradient_hsv(target, half, c2, num - 1, c3, direction);
}

static inline void rgb_fill_gradient4_hsv(rgb_t *target, size_t num, hsv_t c1, hsv_t c2, hsv_t c3, hsv_t c4,
        color_gradient_direction_t direction)
{
    size_t onethird = num / 3;
    size_t twothirds = num * 2 / 3;
    rgb_fill_gradient_hsv(target, 0,         c1, onethird,  c2, direction);
    rgb_fill_gradient_hsv(target, onethird,  c2, twothirds, c3, direction);
    rgb_fill_gradient_hsv(target, twothirds, c3, num - 1,   c4, direction);
}

/**
 * @brief Fill a range of LEDs with a smooth RGB gradient
 *        between two specified RGB colors.
 *
 * Unlike HSV, there is no 'color wheel' in RGB space, and therefore
 * there's only one 'direction' for the gradient to go, and no
 * 'direction' is needed.
 */
void rgb_fill_gradient_rgb(rgb_t *leds, size_t startpos, rgb_t startcolor, size_t endpos, rgb_t endcolor);

static inline void rgb_fill_gradient2_rgb(rgb_t *target, size_t num, rgb_t c1, rgb_t c2)
{
    rgb_fill_gradient_rgb(target, 0, c1, num - 1, c2);
}

static inline void rgb_fill_gradient3_rgb(rgb_t *target, size_t num, rgb_t c1, rgb_t c2, rgb_t c3)
{
    size_t half = num / 2;
    rgb_fill_gradient_rgb(target, 0,    c1, half,    c2);
    rgb_fill_gradient_rgb(target, half, c2, num - 1, c3);
}

static inline void rgb_fill_gradient4_rgb(rgb_t *target, size_t num, rgb_t c1, rgb_t c2, rgb_t c3, rgb_t c4)
{
    size_t onethird = num / 3;
    size_t twothirds = num * 2 / 3;
    rgb_fill_gradient_rgb(target, 0,         c1, onethird,  c2);
    rgb_fill_gradient_rgb(target, onethird,  c2, twothirds, c3);
    rgb_fill_gradient_rgb(target, twothirds, c3, num - 1,   c4);
}

////////////////////////////////////////////////////////////////////////////////
// Palette functions

/**
 * Return an HSV color with 'index' from 'palette' (array of HSV colors).
 * Even though palette has lesser than 256 explicily defined entries, you
 * can use an 'index' from 0..255.  The 'pal_size' explicit palette entries will
 * be spread evenly across the 0..255 range, and the intermedate values
 * will be HSV-interpolated between adjacent explicit entries.
 */
hsv_t color_from_palette_hsv(const hsv_t *palette, uint8_t pal_size, uint8_t index, uint8_t brightness, bool blend);

/**
 * Same for RGB palette
 */
rgb_t color_from_palette_rgb(const rgb_t *palette, uint8_t pal_size, uint8_t index, uint8_t brightness, bool blend);

////////////////////////////////////////////////////////////////////////////////
// Filter functions

/**
 * Function which must be provided by the application for use in two-dimensional
 * filter functions.
 */
typedef size_t (*xy_to_offs_cb)(void *ctx, size_t x, size_t y);

/**
 * @brief One-dimensional blur filter.
 *
 * Spreads light to 2 line neighbors.
 *
 *   0 = no spread at all
 *  64 = moderate spreading
 * 172 = maximum smooth, even spreading
 *
 * 173..255 = wider spreading, but increasing flicker
 *
 * Total light is NOT entirely conserved, so many repeated calls to 'blur'
 * will also result in the light fading, eventually all the way to black;
 * this is by design so that it can be used to (slowly) clear the LEDs
 * to black.
 */
void blur1d(rgb_t *leds, size_t num_leds, fract8 blur_amount);

/**
 * @brief Perform a blur1d on each column of a rectangular matrix
 */
void blur_columns(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy, void *ctx);

/**
 * @brief Perform a blur1d on each row of a rectangular matrix
 */
void blur_rows(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy, void *ctx);

/**
 * @brief Two-dimensional blur filter.
 *
 * Spreads light to 8 XY neighbors.
 *
 *   0 = no spread at all
 *  64 = moderate spreading
 * 172 = maximum smooth, even spreading
 *
 * 173..255 = wider spreading, but increasing flicker
 *
 * Total light is NOT entirely conserved, so many repeated calls to 'blur'
 * will also result in the light fading, eventually all the way to black;
 * this is by design so that it can be used to (slowly) clear the LEDs
 * to black.
 */
void blur2d(rgb_t *leds, size_t width, size_t height, fract8 blur_amount, xy_to_offs_cb xy, void *ctx);

////////////////////////////////////////////////////////////////////////////////
// Gamma functions

/**
 * @brief Single gamma adjustment to a single scalar value.
 *
 * Bear in mind that RGB leds have only eight bits per channel of color resolution,
 * and that very small, subtle shadings may not be visible.
 */
uint8_t apply_gamma2brightness(uint8_t brightness, float gamma);

/**
 * @brief Single gamma adjustment to each channel of a RGB color.
 */
rgb_t apply_gamma2rgb(rgb_t c, float gamma);

/**
 * @brief Different gamma adjustments for each channel of a RGB color.
 */
rgb_t apply_gamma2rgb_channels(rgb_t c, float gamma_r, float gamma_g, float gamma_b);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __COLOR_H__ */

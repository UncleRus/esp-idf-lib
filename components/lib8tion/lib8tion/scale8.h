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

#ifndef __INC_LIB8TION_SCALE_H
#define __INC_LIB8TION_SCALE_H

///@ingroup lib8tion

///@defgroup Scaling Scaling functions
/// Fast, efficient 8-bit scaling functions specifically
/// designed for high-performance LED programming.
///
/// Because of the AVR(Arduino) and ARM assembly language
/// implementations provided, using these functions often
/// results in smaller and faster code than the equivalent
/// program using plain "C" arithmetic and logic.
///@{

///  scale one byte by a second one, which is treated as
///  the numerator of a fraction whose denominator is 256
///  In other words, it computes i * (scale / 256)
///  4 clocks AVR with MUL, 2 clocks ARM
LIB8STATIC_ALWAYS_INLINE uint8_t scale8(uint8_t i, fract8 scale)
{
    return (((uint16_t) i) * (1 + (uint16_t) (scale))) >> 8;
}

///  The "video" version of scale8 guarantees that the output will
///  be only be zero if one or both of the inputs are zero.  If both
///  inputs are non-zero, the output is guaranteed to be non-zero.
///  This makes for better 'video'/LED dimming, at the cost of
///  several additional cycles.
LIB8STATIC_ALWAYS_INLINE uint8_t scale8_video(uint8_t i, fract8 scale)
{
    return (((int) i * (int) scale) >> 8) + ((i && scale) ? 1 : 0);
}

/// scale three one byte values by a fourth one, which is treated as
///         the numerator of a fraction whose demominator is 256
///         In other words, it computes r,g,b * (scale / 256)
///
///         THIS FUNCTION ALWAYS MODIFIES ITS ARGUMENTS IN PLACE
LIB8STATIC void nscale8x3(uint8_t *r, uint8_t *g, uint8_t *b, fract8 scale)
{
    uint16_t scale_fixed = scale + 1;
    *r = (((uint16_t) *r) * scale_fixed) >> 8;
    *g = (((uint16_t) *g) * scale_fixed) >> 8;
    *b = (((uint16_t) *b) * scale_fixed) >> 8;
}

/// scale three one byte values by a fourth one, which is treated as
///         the numerator of a fraction whose demominator is 256
///         In other words, it computes r,g,b * (scale / 256), ensuring
/// that non-zero values passed in remain non zero, no matter how low the scale
/// argument.
///
///         THIS FUNCTION ALWAYS MODIFIES ITS ARGUMENTS IN PLACE
LIB8STATIC void nscale8x3_video(uint8_t *r, uint8_t *g, uint8_t *b, fract8 scale)
{
    uint8_t nonzeroscale = (scale != 0) ? 1 : 0;
    *r = (*r == 0) ? 0 : (((int) *r * (int) (scale)) >> 8) + nonzeroscale;
    *g = (*g == 0) ? 0 : (((int) *g * (int) (scale)) >> 8) + nonzeroscale;
    *b = (*b == 0) ? 0 : (((int) *b * (int) (scale)) >> 8) + nonzeroscale;
}

///  scale two one byte values by a third one, which is treated as
///         the numerator of a fraction whose demominator is 256
///         In other words, it computes i,j * (scale / 256)
///
///         THIS FUNCTION ALWAYS MODIFIES ITS ARGUMENTS IN PLACE
LIB8STATIC void nscale8x2(uint8_t *i, uint8_t *j, fract8 scale)
{
    uint16_t scale_fixed = scale + 1;
    *i = (((uint16_t) *i) * scale_fixed) >> 8;
    *j = (((uint16_t) *j) * scale_fixed) >> 8;
}

///  scale two one byte values by a third one, which is treated as
///         the numerator of a fraction whose demominator is 256
///         In other words, it computes i,j * (scale / 256), ensuring
/// that non-zero values passed in remain non zero, no matter how low the scale
/// argument.
///
///         THIS FUNCTION ALWAYS MODIFIES ITS ARGUMENTS IN PLACE
LIB8STATIC void nscale8x2_video(uint8_t *i, uint8_t *j, fract8 scale)
{
    uint8_t nonzeroscale = (scale != 0) ? 1 : 0;
    *i = (*i == 0) ? 0 : (((int) *i * (int) (scale)) >> 8) + nonzeroscale;
    *j = (*j == 0) ? 0 : (((int) *j * (int) (scale)) >> 8) + nonzeroscale;
}

/// scale a 16-bit unsigned value by an 8-bit value,
///         considered as numerator of a fraction whose denominator
///         is 256. In other words, it computes i * (scale / 256)
LIB8STATIC_ALWAYS_INLINE uint16_t scale16by8(uint16_t i, fract8 scale)
{
    return (i * (1 + ((uint16_t) scale))) >> 8;
}

/// scale a 16-bit unsigned value by a 16-bit value,
///         considered as numerator of a fraction whose denominator
///         is 65536. In other words, it computes i * (scale / 65536)

LIB8STATIC uint16_t scale16(uint16_t i, fract16 scale)
{
    return ((uint32_t) (i) * (1 + (uint32_t) (scale))) / 65536;
}
///@}

///@defgroup Dimming Dimming and brightening functions
///
/// Dimming and brightening functions
///
/// The eye does not respond in a linear way to light.
/// High speed PWM'd LEDs at 50% duty cycle appear far
/// brighter then the 'half as bright' you might expect.
///
/// If you want your midpoint brightness leve (128) to
/// appear half as bright as 'full' brightness (255), you
/// have to apply a 'dimming function'.
///@{

/// Adjust a scaling value for dimming
LIB8STATIC uint8_t dim8_raw(uint8_t x)
{
    return scale8(x, x);
}

/// Adjust a scaling value for dimming for video (value will never go below 1)
LIB8STATIC uint8_t dim8_video(uint8_t x)
{
    return scale8_video(x, x);
}

/// Linear version of the dimming function that halves for values < 128
LIB8STATIC uint8_t dim8_lin(uint8_t x)
{
    if (x & 0x80)
    {
        x = scale8(x, x);
    }
    else
    {
        x += 1;
        x /= 2;
    }
    return x;
}

/// inverse of the dimming function, brighten a value
LIB8STATIC uint8_t brighten8_raw(uint8_t x)
{
    uint8_t ix = 255 - x;
    return 255 - scale8(ix, ix);
}

/// inverse of the dimming function, brighten a value
LIB8STATIC uint8_t brighten8_video(uint8_t x)
{
    uint8_t ix = 255 - x;
    return 255 - scale8_video(ix, ix);
}

/// inverse of the dimming function, brighten a value
LIB8STATIC uint8_t brighten8_lin(uint8_t x)
{
    uint8_t ix = 255 - x;
    if (ix & 0x80)
    {
        ix = scale8(ix, ix);
    }
    else
    {
        ix += 1;
        ix /= 2;
    }
    return 255 - ix;
}

///@}
#endif

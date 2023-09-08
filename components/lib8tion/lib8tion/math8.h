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

#ifndef __INC_LIB8TION_MATH_H
#define __INC_LIB8TION_MATH_H

///@ingroup lib8tion

///@defgroup Math Basic math operations
/// Fast, efficient 8-bit math functions specifically
/// designed for high-performance LED programming.
///@{

/// add one byte to another, saturating at 0xFF
/// @param i - first byte to add
/// @param j - second byte to add
/// @returns the sum of i & j, capped at 0xFF
LIB8STATIC_ALWAYS_INLINE uint8_t qadd8(uint8_t i, uint8_t j)
{
    unsigned int t = i + j;
    if (t > 255)
        t = 255;
    return t;
}

/// Add one byte to another, saturating at 0x7F and -0x80
/// @param i - first byte to add
/// @param j - second byte to add
/// @returns the sum of i & j, capped at 0xFF
LIB8STATIC_ALWAYS_INLINE int8_t qadd7(int8_t i, int8_t j)
{
    int16_t t = i + j;
    if (t > 127)
        t = 127;
    else if (t < -128)
        t = -127;
    return t;
}

/// subtract one byte from another, saturating at 0x00
/// @returns i - j with a floor of 0
LIB8STATIC_ALWAYS_INLINE uint8_t qsub8(uint8_t i, uint8_t j)
{
    int t = i - j;
    if (t < 0)
        t = 0;
    return t;
}

/// add one byte to another, with one byte result
LIB8STATIC_ALWAYS_INLINE uint8_t add8(uint8_t i, uint8_t j)
{
    int t = i + j;
    return t;
}

/// add one byte to two bytes, with two bytes result
LIB8STATIC_ALWAYS_INLINE uint16_t add8to16(uint8_t i, uint16_t j)
{
    uint16_t t = i + j;
    return t;
}

/// subtract one byte from another, 8-bit result
LIB8STATIC_ALWAYS_INLINE uint8_t sub8(uint8_t i, uint8_t j)
{
    int t = i - j;
    return t;
}

/// Calculate an integer average of two unsigned
///       8-bit integer values (uint8_t).
///       Fractional results are rounded down, e.g. avg8(20,41) = 30
LIB8STATIC_ALWAYS_INLINE uint8_t avg8(uint8_t i, uint8_t j)
{
    return (i + j) >> 1;
}

/// Calculate an integer average of two unsigned
///       16-bit integer values (uint16_t).
///       Fractional results are rounded down, e.g. avg16(20,41) = 30
LIB8STATIC_ALWAYS_INLINE uint16_t avg16(uint16_t i, uint16_t j)
{
    return (uint32_t)((uint32_t)(i) + (uint32_t)(j)) >> 1;
}

/// Calculate an integer average of two unsigned
///       8-bit integer values (uint8_t).
///       Fractional results are rounded up, e.g. avg8r(20,41) = 31
LIB8STATIC_ALWAYS_INLINE uint8_t avg8r(uint8_t i, uint8_t j)
{
    return (i + j + 1) >> 1;
}

/// Calculate an integer average of two unsigned
///       16-bit integer values (uint16_t).
///       Fractional results are rounded up, e.g. avg16r(20,41) = 31
LIB8STATIC_ALWAYS_INLINE uint16_t avg16r(uint16_t i, uint16_t j)
{
    return (uint32_t)((uint32_t)(i) + (uint32_t)(j) + 1) >> 1;
}

/// Calculate an integer average of two signed 7-bit
///       integers (int8_t)
///       If the first argument is even, result is rounded down.
///       If the first argument is odd, result is rounded up.
LIB8STATIC_ALWAYS_INLINE int8_t avg7(int8_t i, int8_t j)
{
    return (i >> 1) + (j >> 1) + (i & 0x1);
}

/// Calculate an integer average of two signed 15-bit
///       integers (int16_t)
///       If the first argument is even, result is rounded down.
///       If the first argument is odd, result is rounded up.
LIB8STATIC_ALWAYS_INLINE int16_t avg15(int16_t i, int16_t j)
{
    return (i >> 1) + (j >> 1) + (i & 0x1);
}

///       Calculate the remainder of one unsigned 8-bit
///       value divided by anoter, aka A % M.
///       Implemented by repeated subtraction, which is
///       very compact, and very fast if A is 'probably'
///       less than M.  If A is a large multiple of M,
///       the loop has to execute multiple times.  However,
///       even in that case, the loop is only two
///       instructions long on AVR, i.e., quick.
LIB8STATIC_ALWAYS_INLINE uint8_t mod8(uint8_t a, uint8_t m)
{
    while (a >= m)
        a -= m;
    return a;
}

///          Add two numbers, and calculate the modulo
///          of the sum and a third number, M.
///          In other words, it returns (A+B) % M.
///          It is designed as a compact mechanism for
///          incrementing a 'mode' switch and wrapping
///          around back to 'mode 0' when the switch
///          goes past the end of the available range.
///          e.g. if you have seven modes, this switches
///          to the next one and wraps around if needed:
///            mode = addmod8( mode, 1, 7);
///LIB8STATIC_ALWAYS_INLINESee 'mod8' for notes on performance.
LIB8STATIC uint8_t addmod8(uint8_t a, uint8_t b, uint8_t m)
{
    a += b;
    while (a >= m)
        a -= m;
    return a;
}

///          Subtract two numbers, and calculate the modulo
///          of the difference and a third number, M.
///          In other words, it returns (A-B) % M.
///          It is designed as a compact mechanism for
///          incrementing a 'mode' switch and wrapping
///          around back to 'mode 0' when the switch
///          goes past the end of the available range.
///          e.g. if you have seven modes, this switches
///          to the next one and wraps around if needed:
///            mode = addmod8( mode, 1, 7);
///LIB8STATIC_ALWAYS_INLINESee 'mod8' for notes on performance.
LIB8STATIC uint8_t submod8(uint8_t a, uint8_t b, uint8_t m)
{
    a -= b;
    while (a >= m)
        a -= m;
    return a;
}

/// 8x8 bit multiplication, with 8 bit result
LIB8STATIC_ALWAYS_INLINE uint8_t mul8(uint8_t i, uint8_t j)
{
    return ((int)i * (int)(j)) & 0xFF;
}

/// saturating 8x8 bit multiplication, with 8 bit result
/// @returns the product of i * j, capping at 0xFF
LIB8STATIC_ALWAYS_INLINE uint8_t qmul8(uint8_t i, uint8_t j)
{
    unsigned p = (unsigned)i * (unsigned)j;
    if (p > 255)
        p = 255;
    return p;
}

/// take abs() of a signed 8-bit uint8_t
LIB8STATIC_ALWAYS_INLINE int8_t abs8(int8_t i)
{
    if (i < 0)
        i = -i;
    return i;
}

///         square root for 16-bit integers
///         About three times faster and five times smaller
///         than Arduino's general sqrt on AVR.
LIB8STATIC uint8_t sqrt16(uint16_t x)
{
    if (x <= 1)
        return x;

    uint8_t low = 1; // lower bound
    uint8_t hi, mid;

    hi = x > 7904 ? 255 : (x >> 5) + 8; // initial estimate for upper bound

    do
    {
        mid = (low + hi) >> 1;
        if ((uint16_t)(mid * mid) > x)
        {
            hi = mid - 1;
        }
        else
        {
            if (mid == 255)
                return 255;
            low = mid + 1;
        }
    } while (hi >= low);

    return low - 1;
}

/// blend a variable proportion(0-255) of one byte to another
/// @param a - the starting byte value
/// @param b - the byte value to blend toward
/// @param amountOfB - the proportion (0-255) of b to blend
/// @returns a byte value between a and b, inclusive
LIB8STATIC uint8_t blend8(uint8_t a, uint8_t b, uint8_t amountOfB)
{
    // The BLEND_FIXED formula is
    //
    //   result = (  A*(amountOfA) + B*(amountOfB)              )/ 256
    //
    // …where amountOfA = 255-amountOfB.
    //
    // This formula will never return 255, which is why the BLEND_FIXED + SCALE8_FIXED version is
    //
    //   result = (  A*(amountOfA) + A + B*(amountOfB) + B      ) / 256
    //
    // We can rearrange this formula for some great optimisations.
    //
    //   result = (  A*(amountOfA) + A + B*(amountOfB) + B      ) / 256
    //          = (  A*(255-amountOfB) + A + B*(amountOfB) + B  ) / 256
    //          = (  A*(256-amountOfB) + B*(amountOfB) + B      ) / 256
    //          = (  A*256 + B + B*(amountOfB) - A*(amountOfB)  ) / 256  // this is the version used in SCALE8_FIXED AVR below
    //          = (  A*256 + B + (B-A)*(amountOfB)              ) / 256  // this is the version used in SCALE8_FIXED C below

    uint16_t partial;
    uint8_t result;

    uint8_t amountOfA = 255 - amountOfB;

    partial = (a * amountOfA);
    partial += a;

    partial += (b * amountOfB);
    partial += b;

    result = partial >> 8;

    return result;
}

///@}
#endif

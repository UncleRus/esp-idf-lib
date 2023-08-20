/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Harm Aldick
 *               2021 Tomoyuki Sakurai <y@trombik.org>
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

#ifndef __LED_EFFECT__H__
#define __LED_EFFECT__H__

#include <color.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * returns 32 bit color code in a position of colors. the MSB 8bit is always
 * 0x00.
 *
 * 0xWW 0xRR 0xGG 0xBB
 */
uint32_t led_effect_color_wheel(uint8_t pos);

/**
 * @brief Same as in `led_effect_color_wheel` but Returns `rgb_t`.
 * @param pos Previous position in the wheel
 * @return the next color.
 */
rgb_t led_effect_color_wheel_rgb(uint8_t pos);

#ifdef __cplusplus
}
#endif

#endif

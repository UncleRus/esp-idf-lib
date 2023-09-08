/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file max7219_priv.h
 *
 * ESP-IDF driver for MAX7219/MAX7221
 * Serially Interfaced, 8-Digit LED Display Drivers
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2017, 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MAX7219_PRIV_H__
#define __MAX7219_PRIV_H__

static const uint8_t font_7seg[] = {
    /*  ' '   !     "     #     $     %     &     '     (     )     */
        0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x02, 0x4e, 0x78,
    /*  *     +     ,     -     .     /     0     1     2     3     */
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x7e, 0x30, 0x6d, 0x79,
    /*  4     5     6     7     8     9     :     ;     <     =     */
        0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b, 0x00, 0x00, 0x0d, 0x09,
    /*  >     ?     @     A     B     C     D     E     F     G     */
        0x19, 0x65, 0x00, 0x77, 0x1f, 0x4e, 0x3d, 0x4f, 0x47, 0x5e,
    /*  H     I     J     K     L     M     N     O     P     Q     */
        0x37, 0x06, 0x38, 0x57, 0x0e, 0x76, 0x15, 0x1d, 0x67, 0x73,
    /*  R     S     T     U     V     W     X     Y     Z     [     */
        0x05, 0x5b, 0x0f, 0x1c, 0x3e, 0x2a, 0x49, 0x3b, 0x6d, 0x4e,
    /*  \     ]     ^     _     `     a     b     c     d     e     */
        0x00, 0x78, 0x00, 0x08, 0x02, 0x77, 0x1f, 0x4e, 0x3d, 0x4f,
    /*  f     g     h     i     j     k     l     m     n     o     */
        0x47, 0x5e, 0x37, 0x06, 0x38, 0x57, 0x0e, 0x76, 0x15, 0x1d,
    /*  p     q     r     s     t     u     v     w     x     y     */
        0x67, 0x73, 0x05, 0x5b, 0x0f, 0x1c, 0x3e, 0x2a, 0x49, 0x3b,
    /*  z     {     |     }     ~     */
        0x6d, 0x4e, 0x06, 0x78, 0x00
};


#endif /* __MAX7219_PRIV_H__ */

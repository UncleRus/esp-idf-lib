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

/// This allows testing a RGB for zero-ness
static inline bool hsv_is_zero(hsv_t a)
{
    return a.h || a.s || a.v;
}

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __COLOR_HSV_H__ */

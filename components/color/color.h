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

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __COLOR_H__ */

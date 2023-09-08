/**
 * @file waterfall.h
 *
 * @defgroup led_effect_waterfall led_effect_waterfall
 * @{
 *
 * Waterfall/Fire effect
 *
 * Parameters:
 *     - hue:      Basic hue for waterfall palette. Ignored when mode = WATERFALL_FIRE
 *     - cooling:  How much does the air cool as it rises. Less cooling = taller flames,
 *                 more cooling = shorter flames. Suggested range 20-100.
 *     - sparking: Chance (out of 255) that a new spark will light up. Suggested range 50-200.
 *
 * Recommended parameters for fire mode: cooling = 90, sparking = 80
 */
#ifndef __LED_EFFECTS_WATERFALL_H__
#define __LED_EFFECTS_WATERFALL_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WATERFALL_SIMPLE = 0,
    WATERFALL_COLORS,
    WATERFALL_FIRE,
    WATERFALL_COLD_FIRE,
} led_effect_waterfall_mode_t;

esp_err_t led_effect_waterfall_init(framebuffer_t *fb, led_effect_waterfall_mode_t mode,
        uint8_t hue, uint8_t cooling, uint8_t sparking);

esp_err_t led_effect_waterfall_done(framebuffer_t *fb);

esp_err_t led_effect_waterfall_set_params(framebuffer_t *fb, led_effect_waterfall_mode_t mode,
        uint8_t hue, uint8_t cooling, uint8_t sparking);

esp_err_t led_effect_waterfall_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_WATERFALL_H__ */

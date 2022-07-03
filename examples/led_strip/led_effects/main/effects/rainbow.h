/**
 * @file rainbow.h
 *
 * @defgroup led_effect_rainbow led_effect_rainbow
 * @{
 *
 * Simple rainbow effect
 *
 * Parameters:
 *     - scale:  Density of rainbows. Suggested range 10-50.
 *     - speed:  Speed with which the rainbow shimmers. Suggested range 1-50.
 */
#ifndef __LED_EFFECTS_RAINBOW_H__
#define __LED_EFFECTS_RAINBOW_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RAINBOW_HORIZONTAL = 0,
    RAINBOW_VERTICAL,
    RAINBOW_DIAGONAL,
} led_effect_rainbow_direction_t;

esp_err_t led_effect_rainbow_init(framebuffer_t *fb, led_effect_rainbow_direction_t direction,
        uint8_t scale, uint8_t speed);

esp_err_t led_effect_rainbow_done(framebuffer_t *fb);

esp_err_t led_effect_rainbow_set_params(framebuffer_t *fb, led_effect_rainbow_direction_t direction,
        uint8_t scale, uint8_t speed);

esp_err_t led_effect_rainbow_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_RAINBOW_H__ */

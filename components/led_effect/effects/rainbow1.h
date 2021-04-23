/**
 * @file rainbow1.h
 *
 * @defgroup led_effect_rainbow1 led_effect_rainbow1
 * @{
 *
 * Simple rainbow effect
 *
 * Parameters:
 *     - scale:  Density of rainbows. Suggested range 10-50.
 *     - speed:  Speed with which the rainbow shimmers. Suggested range 1-50.
 */
#ifndef __LED_EFFECTS_RAINBOW1_H__
#define __LED_EFFECTS_RAINBOW1_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RAINBOW1_HORIZONTAL = 0,
    RAINBOW1_VERTICAL,
    RAINBOW1_DIAGONAL,
} led_effect_rainbow1_direction_t;

esp_err_t led_effect_rainbow1_init(framebuffer_t *fb, led_effect_rainbow1_direction_t direction,
        uint8_t scale, uint8_t speed);

esp_err_t led_effect_rainbow1_done(framebuffer_t *fb);

esp_err_t led_effect_rainbow1_set_params(framebuffer_t *fb, led_effect_rainbow1_direction_t direction,
        uint8_t scale, uint8_t speed);

esp_err_t led_effect_rainbow1_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_RAINBOW1_H__ */

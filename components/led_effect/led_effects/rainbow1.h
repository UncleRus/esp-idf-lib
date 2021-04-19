#ifndef __LED_EFFECTS_RAINBOW1_H__
#define __LED_EFFECTS_RAINBOW1_H__

#include <led_effect.h>

typedef enum {
    RAINBOW1_HORIZONTAL = 0,
    RAINBOW1_VERTICAL,
} led_effect_rainbow1_direction_t;

esp_err_t led_effect_rainbow1_init(led_effect_t *state, led_effect_rainbow1_direction_t direction,
        uint8_t scale, uint8_t speed);

esp_err_t led_effect_rainbow1_done(led_effect_t *state);

esp_err_t led_effect_rainbow1_set_params(led_effect_t *state, led_effect_rainbow1_direction_t direction,
        uint8_t scale, uint8_t speed);

esp_err_t led_effect_rainbow1_run(led_effect_t *state);

#endif /* __LED_EFFECTS_RAINBOW1_H__ */

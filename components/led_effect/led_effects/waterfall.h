#ifndef __LED_EFFECTS_WATERFALL_H__
#define __LED_EFFECTS_WATERFALL_H__

#include <led_effect.h>

typedef enum {
    WATERFALL_SIMPLE = 0,
    WATERFALL_COLORS,
    WATERFALL_FIRE,
} led_effect_waterfall_mode_t;

esp_err_t led_effect_waterfall_init(led_effect_t *state, led_effect_waterfall_mode_t mode,
        uint8_t hue, uint8_t cooling, uint8_t sparking);

esp_err_t led_effect_waterfall_done(led_effect_t *state);

esp_err_t led_effect_waterfall_set_params(led_effect_t *state, led_effect_waterfall_mode_t mode,
        uint8_t hue, uint8_t cooling, uint8_t sparking);

esp_err_t led_effect_waterfall_run(led_effect_t *state);

#endif /* __LED_EFFECTS_WATERFALL_H__ */

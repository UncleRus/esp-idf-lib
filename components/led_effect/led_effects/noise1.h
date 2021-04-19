#ifndef __LED_EFFECTS_NOISE1_H__
#define __LED_EFFECTS_NOISE1_H__

#include <led_effect.h>

esp_err_t led_effect_noise1_init(led_effect_t *state, uint8_t scale, uint8_t speed);

esp_err_t led_effect_noise1_done(led_effect_t *state);

esp_err_t led_effect_noise1_set_params(led_effect_t *state, uint8_t scale, uint8_t speed);

esp_err_t led_effect_noise1_run(led_effect_t *state);

#endif /* __LED_EFFECTS_NOISE1_H__ */

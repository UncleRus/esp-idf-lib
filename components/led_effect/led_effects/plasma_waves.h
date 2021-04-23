/**
 * @file plasma_waves.h
 *
 * @defgroup led_effect_plasma_waves led_effect_plasma_waves
 * @{
 *
 * Plasma waves effect
 *
 * Author: Edmund "Skorn" Horn
 */
#ifndef __LED_EFFECTS_PLASMA_WAVES_H__
#define __LED_EFFECTS_PLASMA_WAVES_H__

#include <led_effect.h>

esp_err_t led_effect_plasma_waves_init(led_effect_t *state, uint8_t speed);

esp_err_t led_effect_plasma_waves_done(led_effect_t *state);

esp_err_t led_effect_plasma_waves_set_params(led_effect_t *state, uint8_t speed);

esp_err_t led_effect_plasma_waves_run(led_effect_t *state);

/**@}*/

#endif /* __LED_EFFECTS_PLASMA_WAVES_H__ */

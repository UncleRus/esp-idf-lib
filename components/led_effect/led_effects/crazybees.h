/**
 * @file crazybees.h
 *
 * @defgroup led_effect_crazybees led_effect_crazybees
 * @{
 *
 * Crazy Bees effect
 *
 * Author: stepko
 */
#ifndef __LED_EFFECTS_CRAZYBEES_H__
#define __LED_EFFECTS_CRAZYBEES_H__

#include <led_effect.h>

#define CRAZYBEES_MAX_BEES 10

esp_err_t led_effect_crazybees_init(led_effect_t *state, uint8_t num_bees);

esp_err_t led_effect_crazybees_done(led_effect_t *state);

esp_err_t led_effect_crazybees_set_params(led_effect_t *state, uint8_t num_bees);

esp_err_t led_effect_crazybees_run(led_effect_t *state);

/**@}*/

#endif /* __LED_EFFECTS_CRAZYBEES_H__ */

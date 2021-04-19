/**
 * @file dna.h
 *
 * @ingroup effects
 * @{
 *
 * DNA spiral effect, based on Yaroslaw Turbin code (https://vk.com/ldirko, https://www.reddit.com/user/ldirko/)
 * Max effect framebuffer size 256x256
 *
 * Parameters:
 *   speed  - Speed of rotation, 10 - 100
 *   size   - Spiral size, 1 - 10
 *   border - Add white border
 */
#ifndef __LED_EFFECTS_DNA_H__
#define __LED_EFFECTS_DNA_H__

#include <led_effect.h>

esp_err_t led_effect_dna_init(led_effect_t *state, uint8_t speed, uint8_t size, bool border);

esp_err_t led_effect_dna_done(led_effect_t *state);

esp_err_t led_effect_dna_set_params(led_effect_t *state, uint8_t speed, uint8_t size, bool border);

esp_err_t led_effect_dna_run(led_effect_t *state);

/**@}*/

#endif /* __LED_EFFECTS_DNA_H__ */

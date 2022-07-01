/**
 * @file crazybees.h
 *
 * @defgroup led_effect_crazybees led_effect_crazybees
 * @{
 *
 * Crazy Bees effect
 *
 * Author: Stepko
 */
#ifndef __LED_EFFECTS_CRAZYBEES_H__
#define __LED_EFFECTS_CRAZYBEES_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRAZYBEES_MAX_BEES 10

esp_err_t led_effect_crazybees_init(framebuffer_t *fb, uint8_t num_bees);

esp_err_t led_effect_crazybees_done(framebuffer_t *fb);

esp_err_t led_effect_crazybees_set_params(framebuffer_t *fb, uint8_t num_bees);

esp_err_t led_effect_crazybees_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_CRAZYBEES_H__ */

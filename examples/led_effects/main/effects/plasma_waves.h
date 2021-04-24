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

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_effect_plasma_waves_init(framebuffer_t *fb, uint8_t speed);

esp_err_t led_effect_plasma_waves_done(framebuffer_t *fb);

esp_err_t led_effect_plasma_waves_set_params(framebuffer_t *fb, uint8_t speed);

esp_err_t led_effect_plasma_waves_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_PLASMA_WAVES_H__ */

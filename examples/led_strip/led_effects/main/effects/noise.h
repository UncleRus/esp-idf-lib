/**
 * @file noise.h
 *
 * @defgroup led_effect_noise led_effect_noise
 * @{
 *
 * Perlin noise effect
 *
 * Author: Chuck Sommerville
 */
#ifndef __LED_EFFECTS_NOISE_H__
#define __LED_EFFECTS_NOISE_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_effect_noise_init(framebuffer_t *fb, uint8_t scale, uint8_t speed);

esp_err_t led_effect_noise_done(framebuffer_t *fb);

esp_err_t led_effect_noise_set_params(framebuffer_t *fb, uint8_t scale, uint8_t speed);

esp_err_t led_effect_noise_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_NOISE_H__ */

/**
 * @file sparkles1.h
 *
 * @defgroup led_effect_sparkles1 led_effect_sparkles1
 * @{
 *
 * Colored sparkles effect.
 *
 */
#ifndef __LED_EFFECTS_SPARKLES1_H__
#define __LED_EFFECTS_SPARKLES1_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_effect_sparkles1_init(framebuffer_t *fb, uint8_t max_sparkles, uint8_t fadeout_speed);

esp_err_t led_effect_sparkles1_done(framebuffer_t *fb);

esp_err_t led_effect_sparkles1_set_params(framebuffer_t *fb, uint8_t max_sparkles, uint8_t fadeout_speed);

esp_err_t led_effect_sparkles1_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_SPARKLES1_H__ */

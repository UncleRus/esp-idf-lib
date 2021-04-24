/**
 * @file sparkles.h
 *
 * @defgroup led_effect_sparkles led_effect_sparkles
 * @{
 *
 * Colored sparkles effect.
 *
 */
#ifndef __LED_EFFECTS_SPARKLES_H__
#define __LED_EFFECTS_SPARKLES_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_effect_sparkles_init(framebuffer_t *fb, uint8_t max_sparkles, uint8_t fadeout_speed);

esp_err_t led_effect_sparkles_done(framebuffer_t *fb);

esp_err_t led_effect_sparkles_set_params(framebuffer_t *fb, uint8_t max_sparkles, uint8_t fadeout_speed);

esp_err_t led_effect_sparkles_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_SPARKLES_H__ */

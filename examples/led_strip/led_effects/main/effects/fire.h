/**
 * @file fire.h
 *
 * @defgroup led_effect_fire led_effect_fire
 * @{
 *
 * Fire effect based on Perlin noise
 *
 * Author: Yaroslaw Turbin (https://vk.com/ldirko, https://www.reddit.com/user/ldirko/)
 *
 * https://pastebin.com/jSSVSRi6
 */
#ifndef __LED_EFFECTS_FIRE_H__
#define __LED_EFFECTS_FIRE_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FIRE_PALETTE_FIRE = 0,
    FIRE_PALETTE_BLUE,
    FIRE_PALETTE_GREEN
} led_effect_fire_palette_t;

esp_err_t led_effect_fire_init(framebuffer_t *fb, led_effect_fire_palette_t p);

esp_err_t led_effect_fire_set_params(framebuffer_t *fb, led_effect_fire_palette_t p);

esp_err_t led_effect_fire_done(framebuffer_t *fb);

esp_err_t led_effect_fire_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_FIRE_H__ */

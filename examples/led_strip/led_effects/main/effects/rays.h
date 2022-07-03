/**
 * @file rays.h
 *
 * Colored rays effect
 *
 * Author: Yaroslaw Turbin (https://vk.com/ldirko, https://www.reddit.com/user/ldirko/)
 *
 * https://editor.soulmatelights.com/gallery/819-colored-bursts
 *
 * Max supported framebuffer size is 256x256
 *
 * Parameters:
 *   - speed:    Speed of rays movement, 0 - 50
 *   - min_rays: Minimal rays count, 1 - 10
 *   - max_rays: Maximal rays count, 10 - 20
 */
#ifndef __LED_EFFECTS_RAYS_H__
#define __LED_EFFECTS_RAYS_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_effect_rays_init(framebuffer_t *fb, uint8_t speed, uint8_t min_rays, uint8_t max_rays);

esp_err_t led_effect_rays_done(framebuffer_t *fb);

esp_err_t led_effect_rays_set_params(framebuffer_t *fb, uint8_t speed, uint8_t min_rays, uint8_t max_rays);

esp_err_t led_effect_rays_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_RAYS_H__ */

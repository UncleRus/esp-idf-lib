/**
 * @file rays.h
 *
 * @defgroup led_effect_rays led_effect_rays
 * @{
 *
 * Colored rays effect, based on Yaroslaw Turbin code (https://vk.com/ldirko, https://www.reddit.com/user/ldirko/)
 * https://editor.soulmatelights.com/gallery/819-colored-bursts
 *
 * Max supported framebuffer size 256x256
 *
 * Parameters:
 *   - speed:    Speed of rays movement, 0 - 50
 *   - min_rays: Minimal rays count, 1 - 10
 *   - max_rays: Maximal rays count, 10 - 20
 */
#ifndef __LED_EFFECTS_RAYS_H__
#define __LED_EFFECTS_RAYS_H__

#include <led_effect.h>

esp_err_t led_effect_rays_init(led_effect_t *state, uint8_t speed, uint8_t min_rays, uint8_t max_rays);

esp_err_t led_effect_rays_done(led_effect_t *state);

esp_err_t led_effect_rays_set_params(led_effect_t *state, uint8_t speed, uint8_t min_rays, uint8_t max_rays);

esp_err_t led_effect_rays_run(led_effect_t *state);

/**@}*/

#endif /* __LED_EFFECTS_RAYS_H__ */

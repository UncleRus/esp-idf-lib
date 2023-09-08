/**
 * @file dna.h
 *
 * @defgroup led_effect_dna led_effect_dna
 * @{
 *
 * DNA spiral effect
 *
 * Author: Yaroslaw Turbin (https://vk.com/ldirko, https://www.reddit.com/user/ldirko/)
 *
 * Max supported framebuffer size is 256x256
 *
 * Parameters:
 *   - speed:  Speed of rotation, 10 - 100
 *   - size:   Spiral size, 1 - 10
 *   - border: Add white border
 */
#ifndef __LED_EFFECTS_DNA_H__
#define __LED_EFFECTS_DNA_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_effect_dna_init(framebuffer_t *fb, uint8_t speed, uint8_t size, bool border);

esp_err_t led_effect_dna_done(framebuffer_t *fb);

esp_err_t led_effect_dna_set_params(framebuffer_t *fb, uint8_t speed, uint8_t size, bool border);

esp_err_t led_effect_dna_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_DNA_H__ */

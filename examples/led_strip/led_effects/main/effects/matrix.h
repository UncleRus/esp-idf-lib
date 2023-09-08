/**
 * @file matrix.h
 *
 * @defgroup led_effect_matrix led_effect_matrix
 * @{
 *
 * Matrix effect
 *
 */
#ifndef __LED_EFFECTS_MATRIX_H__
#define __LED_EFFECTS_MATRIX_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t led_effect_matrix_init(framebuffer_t *fb, uint8_t density);

esp_err_t led_effect_matrix_done(framebuffer_t *fb);

esp_err_t led_effect_matrix_set_params(framebuffer_t *fb, uint8_t density);

esp_err_t led_effect_matrix_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_MATRIX_H__ */

/**
 * @file rain.h
 *
 * @defgroup led_effect_rain led_effect_rain
 * @{
 *
 * Rain effect by Shaitan
 *
 */
#ifndef __LED_EFFECTS_RAIN_H__
#define __LED_EFFECTS_RAIN_H__

#include <framebuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RAIN_MODE_SINGLE_COLOR = 0,
    RAIN_MODE_RAINBOW
} led_effect_rain_mode_t;

esp_err_t led_effect_rain_init(framebuffer_t *fb, led_effect_rain_mode_t mode, uint8_t hue, uint8_t density, uint8_t tail);

esp_err_t led_effect_rain_done(framebuffer_t *fb);

esp_err_t led_effect_rain_set_params(framebuffer_t *fb, led_effect_rain_mode_t mode, uint8_t hue, uint8_t density, uint8_t tail);

esp_err_t led_effect_rain_run(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECTS_RAIN_H__ */

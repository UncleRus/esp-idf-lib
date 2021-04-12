/**
 * Simple abstract animation on addressable LED matrix
 */
#ifndef __LED_ANIMATION_H__
#define __LED_ANIMATION_H__

#include <esp_err.h>
#include <color.h>

typedef struct
{
    size_t frame_num;
    rgb_t *frame_buf;
    size_t width;
    size_t height;
    uint8_t buf[64];
} led_animation_t;

esp_err_t led_animation_init(led_animation_t *state, size_t width, size_t height);
esp_err_t led_animation_free(led_animation_t *state);
esp_err_t led_animation_set_pixel(led_animation_t *state, size_t x, size_t y, rgb_t color);
esp_err_t led_animation_get_pixel(led_animation_t *state, size_t x, size_t y, rgb_t *color);
esp_err_t led_animation_render(led_animation_t *state, led_strip_t *strip);

#endif /* __LED_ANIMATION_H__ */

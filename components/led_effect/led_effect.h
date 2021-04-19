/**
 * Simple abstraction of effect on an addressable LED matrix
 */
#ifndef __LED_EFFECT_H__
#define __LED_EFFECT_H__

#include <esp_err.h>
#include <color.h>

#define LED_EFFECT_INTERNAL_BUF_SIZE 256

#ifdef __cplusplus
extern "C" {
#endif

#define LED_EFFECT_FRAME_BUF_OFFS(state, x, y) (((state)->width * (y) + (x)) * ((state)->buf_type == LED_EFFECT_RGB ? sizeof(rgb_t) : sizeof(hsv_t)))

typedef enum {
    LED_EFFECT_RGB = 0,
    LED_EFFECT_HSV
} led_effect_type_t;

typedef struct led_effect_s led_effect_t;

typedef esp_err_t (*led_effect_render_cb_t)(led_effect_t *state, void *arg);

struct led_effect_s
{
    led_effect_type_t buf_type;
    uint8_t *frame_buf;
    size_t width;
    size_t height;
    size_t frame_num;
    uint64_t last_frame_us;
    led_effect_render_cb_t render;
    uint8_t *internal;  // for effect settings, internal vars, palettes and so on
};

/**
 * @brief Initialize effect state
 *
 * @param state     Effect state
 * @param width     Frame width in pixels
 * @param height    Frame height in pixels
 * @param buf_type  Color type of frame buffer
 * @param render_cb Renderer callback function
 *
 * @return          ESP_OK on success
 */
esp_err_t led_effect_init(led_effect_t *state, size_t width, size_t height, led_effect_type_t buf_type,
        led_effect_render_cb_t render_cb);

/**
 * @brief Free effect state buffers
 *
 * @param state     Effect state
 * @return          ESP_OK on success
 */
esp_err_t led_effect_free(led_effect_t *state);

esp_err_t led_effect_set_pixel_rgb(led_effect_t *state, size_t x, size_t y, rgb_t color);

esp_err_t led_effect_set_pixel_hsv(led_effect_t *state, size_t x, size_t y, hsv_t color);

esp_err_t led_effect_get_pixel_rgb(led_effect_t *state, size_t x, size_t y, rgb_t *color);

esp_err_t led_effect_get_pixel_hsv(led_effect_t *state, size_t x, size_t y, hsv_t *color);

esp_err_t led_effect_clear(led_effect_t *state);

esp_err_t led_effect_end_frame(led_effect_t *state);

esp_err_t led_effect_render(led_effect_t *state, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* __LED_EFFECT_H__ */

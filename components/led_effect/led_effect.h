/**
 * @file led_effect.h
 * @defgroup led_effect led_effect
 * @{
 *
 * Simple abstraction of effects on an addressable LED matrix for ESP-IDF
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 *
 */
#ifndef __LED_EFFECT_H__
#define __LED_EFFECT_H__

#include <esp_err.h>
#include <color.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_EFFECT_FRAMEBUF_OFFS(state, x, y) ((state)->width * (y) + (x))

typedef enum {
    SHIFT_LEFT  = 0,
    SHIFT_RIGHT,
    SHIFT_UP,
    SHIFT_DOWN
} led_effect_shift_direction_t;

typedef struct led_effect_s led_effect_t;

/**
 * Renderer callback prototype
 */
typedef esp_err_t (*led_effect_render_cb_t)(led_effect_t *state, void *arg);

/**
 * Effect state descriptor
 */
struct led_effect_s
{
    rgb_t *frame_buf;              ///< RGB framebuffer
    size_t width;                  ///< Framebuffer width
    size_t height;                 ///< Framebuffer height
    size_t frame_num;              ///< Number of rendered frames
    uint64_t last_frame_us;        ///< Time of last rendered frame since boot in microseconds
    volatile bool busy;            ///< true if rendering
    led_effect_render_cb_t render; ///< See ::led_effect_render_frame()
    uint8_t *internal;             ///< Buffer for effect settings, internal vars, palettes and so on
};

/**
 * @brief Initialize effect state
 *
 * @param state     Effect state
 * @param width     Frame width in pixels
 * @param height    Frame height in pixels
 * @param render_cb Renderer callback function
 *
 * @return          ESP_OK on success
 */
esp_err_t led_effect_init(led_effect_t *state, size_t width, size_t height, led_effect_render_cb_t render_cb);

/**
 * @brief Free effect state buffers
 *
 * @param state     Effect state
 * @return          ESP_OK on success
 */
esp_err_t led_effect_free(led_effect_t *state);

/**
 * @brief Render frambuffer to actual display or LED strip
 *
 * Rendering is performed by calling the callback function with passing
 * it as arguments \p state and \p render_ctx
 *
 * @param state      Effect state
 * @param render_ctx Argument to pass to callback
 * @return           ESP_OK on success
 */
esp_err_t led_effect_render_frame(led_effect_t *state, void *render_ctx);

/**
 * @name Functions to use in effects
 */

/**
 * @brief Set RGB color of framebuffer pixel
 *
 * If the framebuffer color type is HSV, the color will be converted automatically
 *
 * @param state     Effect state
 * @param x         X coordinate
 * @param y         Y coordinate
 * @param color     RGB color
 * @return          ESP_OK on success
 */
esp_err_t led_effect_set_pixel_rgb(led_effect_t *state, size_t x, size_t y, rgb_t color);

/**
 * @brief Set HSV color of framebuffer pixel
 *
 * If the framebuffer color type is RGB, the color will be converted automatically
 *
 * @param state     Effect state
 * @param x         X coordinate
 * @param y         Y coordinate
 * @param color     HSV color
 * @return          ESP_OK on success
 */
esp_err_t led_effect_set_pixel_hsv(led_effect_t *state, size_t x, size_t y, hsv_t color);

/**
 * @brief Get RGB color of framebuffer pixel
 *
 * If the framebuffer color type is HSV, result will be converted to RGB automatically
 *
 * @param state       Effect state
 * @param x           X coordinate
 * @param y           Y coordinate
 * @param[out] color  RGB color
 * @return            ESP_OK on success
 */
esp_err_t led_effect_get_pixel_rgb(led_effect_t *state, size_t x, size_t y, rgb_t *color);

/**
 * @brief Get HSV color of framebuffer pixel
 *
 * If the framebuffer color type is RGB, result will be converted to HSV automatically
 *
 * @param state       Effect state
 * @param x           X coordinate
 * @param y           Y coordinate
 * @param[out] color  HSV color
 * @return            ESP_OK on success
 */
esp_err_t led_effect_get_pixel_hsv(led_effect_t *state, size_t x, size_t y, hsv_t *color);

/**
 * @brief Clear framebuffer
 *
 * @param state     Effect state
 * @return          ESP_OK on success
 */
esp_err_t led_effect_clear(led_effect_t *state);

/**
 * @brief Shift framebuffer
 *
 * @param state     Effect state
 * @param offs      Shift size
 * @param dir       Shift direction
 * @return          ESP_OK on success
 */
esp_err_t led_effect_shift(led_effect_t *state, size_t offs, led_effect_shift_direction_t dir);

/**
 * @brief Fade pixels to black
 *
 * rgb_fade(pixel, scale) for all pixels in framebuffer
 *
 * @param state     Effect state
 * @param scale     Amount of scaling
 * @return          ESP_OK on success
 */
esp_err_t led_effect_fade(led_effect_t *state, uint8_t scale);

/**
 * @brief Start frame rendering
 *
 * This function must be called in effects at the beginning of rendering frame
 *
 * @param state     Effect state
 * @return          ESP_OK on success
 */
esp_err_t led_effect_begin_frame(led_effect_t *state);

/**
 * @brief Finish frame rendering
 *
 * This function must be called in effects at the end of rendering frame
 *
 * @param state     Effect state
 * @return          ESP_OK on success
 */
esp_err_t led_effect_end_frame(led_effect_t *state);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECT_H__ */

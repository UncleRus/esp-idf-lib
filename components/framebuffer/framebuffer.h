/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file framebuffer.h
 * @defgroup framebuffer framebuffer
 * @{
 *
 * Simple abstraction of RGB framebuffer for ESP-IDF
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __FRAMEBUFFER_H__
#define __FRAMEBUFFER_H__

#include <esp_err.h>
#include <color.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FB_OFFSET(fb, x, y) ((fb)->width * (y) + (x))

#define FB_SIZE(fb) ((fb)->width * (fb)->height * sizeof(rgb_t))

typedef enum {
    FB_SHIFT_LEFT  = 0,
    FB_SHIFT_RIGHT,
    FB_SHIFT_UP,
    FB_SHIFT_DOWN
} fb_shift_direction_t;

typedef struct framebuffer_s framebuffer_t;

/**
 * Renderer callback prototype
 */
typedef esp_err_t (*fb_render_cb_t)(framebuffer_t *fb, void *arg);

/**
 * Framebuffer descriptor descriptor
 */
struct framebuffer_s
{
    rgb_t *data;                   ///< RGB framebuffer
    size_t width;                  ///< Framebuffer width
    size_t height;                 ///< Framebuffer height
    size_t frame_num;              ///< Number of rendered frames
    uint64_t last_frame_us;        ///< Time of last rendered frame since boot in microseconds
    fb_render_cb_t render;         ///< See ::fb_render()
    uint8_t *internal;             ///< Buffer for effect settings, internal vars, palettes and so on
    SemaphoreHandle_t mutex;
};

/**
 * @brief Initialize framebuffer
 *
 * @param fb        Framebuffer descriptor
 * @param width     Frame width in pixels
 * @param height    Frame height in pixels
 * @param render_cb Renderer callback function
 *
 * @return          ESP_OK on success
 */
esp_err_t fb_init(framebuffer_t *fb, size_t width, size_t height, fb_render_cb_t render_cb);

/**
 * @brief Free Framebuffer descriptor buffers
 *
 * @param fb     Framebuffer descriptor
 * @return          ESP_OK on success
 */
esp_err_t fb_free(framebuffer_t *fb);

/**
 * @brief Render frambuffer to actual display or LED strip
 *
 * Rendering is performed by calling the callback function with passing
 * it as arguments \p fb and \p ctx
 *
 * @param fb   Framebuffer descriptor
 * @param ctx  Argument to pass to callback
 * @return     ESP_OK on success
 */
esp_err_t fb_render(framebuffer_t *fb, void *ctx);

/**
 * @brief Set RGB color of framebuffer pixel
 *
 * @param fb        Framebuffer descriptor
 * @param x         X coordinate
 * @param y         Y coordinate
 * @param color     RGB color
 * @return          ESP_OK on success
 */
esp_err_t fb_set_pixel_rgb(framebuffer_t *fb, size_t x, size_t y, rgb_t color);

/**
 * @brief Set HSV color of framebuffer pixel
 *
 * @param fb        Framebuffer descriptor
 * @param x         X coordinate
 * @param y         Y coordinate
 * @param color     HSV color
 * @return          ESP_OK on success
 */
esp_err_t fb_set_pixel_hsv(framebuffer_t *fb, size_t x, size_t y, hsv_t color);

/**
 * @brief Set RGB pixel with subpixel resolution
 *
 * @param fb        Framebuffer descriptor
 * @param x         X coordinate
 * @param y         Y coordinate
 * @param color     RGB color
 * @return          ESP_OK on success
 */
esp_err_t fb_set_pixelf_rgb(framebuffer_t *fb, float x, float y, rgb_t color);

/**
 * @brief Set HSV pixel with subpixel resolution
 *
 * @param fb        Framebuffer descriptor
 * @param x         X coordinate
 * @param y         Y coordinate
 * @param color     HSV color
 * @return          ESP_OK on success
 */
esp_err_t fb_set_pixelf_hsv(framebuffer_t *fb, float x, float y, hsv_t color);

/**
 * @brief Get RGB color of framebuffer pixel
 *
 * @param fb          Framebuffer descriptor
 * @param x           X coordinate
 * @param y           Y coordinate
 * @param[out] color  RGB color
 * @return            ESP_OK on success
 */
esp_err_t fb_get_pixel_rgb(framebuffer_t *fb, size_t x, size_t y, rgb_t *color);

/**
 * @brief Get HSV color of framebuffer pixel
 *
 * @param fb          Framebuffer descriptor
 * @param x           X coordinate
 * @param y           Y coordinate
 * @param[out] color  HSV color
 * @return            ESP_OK on success
 */
esp_err_t fb_get_pixel_hsv(framebuffer_t *fb, size_t x, size_t y, hsv_t *color);

/**
 * @brief Clear framebuffer
 *
 * @param fb     Framebuffer descriptor
 * @return       ESP_OK on success
 */
esp_err_t fb_clear(framebuffer_t *fb);

/**
 * @brief Shift framebuffer
 *
 * @param fb        Framebuffer descriptor
 * @param offs      Shift size
 * @param dir       Shift direction
 * @return          ESP_OK on success
 */
esp_err_t fb_shift(framebuffer_t *fb, size_t offs, fb_shift_direction_t dir);

/**
 * @brief Fade pixels to black
 *
 * rgb_fade(pixel, scale) for all pixels in framebuffer
 *
 * @param fb        Framebuffer descriptor
 * @param scale     Amount of scaling
 * @return          ESP_OK on success
 */
esp_err_t fb_fade(framebuffer_t *fb, uint8_t scale);

/**
 * @brief Aplly two-dimensional blur filter on framebuffer
 *
 * Spreads light to 8 XY neighbors.
 *
 *   0 = no spread at all
 *  64 = moderate spreading
 * 172 = maximum smooth, even spreading
 *
 * 173..255 = wider spreading, but increasing flicker
 *
 * @param fb        Framebuffer descriptor
 * @param amount    Amount of bluring
 * @return          ESP_OK on success
 */
esp_err_t fb_blur2d(framebuffer_t *fb, fract8 amount);

/**
 * @brief Start frame rendering
 *
 * This function must be called in effects at the beginning of rendering frame
 *
 * @param fb     Framebuffer descriptor
 * @return       ESP_OK on success
 */
esp_err_t fb_begin(framebuffer_t *fb);

/**
 * @brief Finish frame rendering
 *
 * This function must be called in effects at the end of rendering frame
 *
 * @param fb     Framebuffer descriptor
 * @return       ESP_OK on success
 */
esp_err_t fb_end(framebuffer_t *fb);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __FRAMEBUFFER_H__ */

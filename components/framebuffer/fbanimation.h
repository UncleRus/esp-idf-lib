/**
 * @file fbanimation.h
 * @defgroup animation animation
 * @{
 *
 * ESP-IDF abstraction of framebuffer animation
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __FBANIMATION_H__
#define __FBANIMATION_H__

#include <esp_timer.h>
#include "framebuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Draw funtion type
 */
typedef esp_err_t (*fb_draw_cb_t)(framebuffer_t *fb);

/**
 * Animation descriptor
 */
typedef struct
{
    framebuffer_t *fb;         ///< Framebuffer descriptor
    void *render_ctx;          ///< Renderer context
    esp_timer_handle_t timer;  ///< Animation timer
    fb_draw_cb_t draw;         ///< Draw function
} fb_animation_t;

/**
 * @brief Create animation based on LED effect
 *
 * @param animation     Animation descriptor
 * @param fb            Framebuffer descriptor
 * @return              ESP_OK on success
 */
esp_err_t fb_animation_init(fb_animation_t *animation, framebuffer_t *fb);

/**
 * @brief Play animation
 *
 * @param animation     Animation descriptor
 * @param fps           Target FPS
 * @param draw          Function for drawing on a framebuffer
 * @param render_ctx    Renderer callback argument
 * @return              ESP_OK on success
 */
esp_err_t fb_animation_play(fb_animation_t *animation, uint8_t fps, fb_draw_cb_t draw, void *render_ctx);

/**
 * @brief Stop playing animation
 *
 * @param animation     Animation descriptor
 * @return              ESP_OK on success
 */
esp_err_t fb_animation_stop(fb_animation_t *animation);

/**
 * @brief Create animation based on LED effect
 *
 * @param animation     Animation descriptor
 * @return              ESP_OK on success
 */
esp_err_t fb_animation_free(fb_animation_t *animation);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __FBANIMATION_H__ */

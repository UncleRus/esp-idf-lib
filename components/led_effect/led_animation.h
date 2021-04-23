/**
 * @file led_effect.h
 * @defgroup led_effect_animation led_effect_animation
 * @{
 *
 * Animation of LED effects
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 *
 */
#ifndef __LED_EFFECT_ANIMATION_H__
#define __LED_EFFECT_ANIMATION_H__

#include "led_effect.h"
#include <esp_timer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*led_effect_run_cb_t)(led_effect_t *state);

/**
 * Animation descriptor
 */
typedef struct
{
    led_effect_t *state;             ///< Effect state descriptor
    void *render_ctx;                ///< Renderer context
    esp_timer_handle_t timer;        ///< Animation timer
    led_effect_run_cb_t effect_func; ///< Effect loop
} led_effect_animation_t;

/**
 * @brief Create animation based on LED effect
 *
 * @param animation    Pointer to animation descriptor
 * @param state        Pointer to LED effect descriptor
 * @return ESP_OK on success
 */
esp_err_t led_effect_animation_init(led_effect_animation_t *animation, led_effect_t *state);

/**
 * @brief Play animation
 *
 * @param animation    Pointer to animation descriptor
 * @param fps          Target FPS
 * @param effect_func  Main effect function
 * @param render_ctx   Renderer callback argument
 * @return ESP_OK on success
 */
esp_err_t led_effect_animation_play(led_effect_animation_t *animation, uint8_t fps,
        led_effect_run_cb_t effect_func, void *render_ctx);

/**
 * @brief Stop animation playing
 *
 * @param animation    Pointer to animation descriptor
 * @return ESP_OK on success
 */
esp_err_t led_effect_animation_stop(led_effect_animation_t *animation);

/**
 * @brief Create animation based on LED effect
 *
 * @param animation    Pointer to animation descriptor
 * @return ESP_OK on success
 */
esp_err_t led_effect_animation_free(led_effect_animation_t *animation);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_EFFECT_ANIMATION_H__ */

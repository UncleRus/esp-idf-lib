/**
 * @file animation.c
 *
 * ESP-IDF abstraction of framebuffer animation
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <esp_err.h>
#include <esp_log.h>
#include "fbanimation.h"

static const char *TAG = "animation";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static void display_frame(void *ctx)
{
    fb_animation_t *animation = (fb_animation_t *)ctx;

    // run effect
    esp_err_t res = animation->draw ? animation->draw(animation->fb) : ESP_FAIL;
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error running effect %d (%s)", res, esp_err_to_name(res));
        return;
    }
    // render frame
    fb_render(animation->fb, animation->render_ctx);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error rendering frame %d (%s)", res, esp_err_to_name(res));
        return;
    }
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t fb_animation_init(fb_animation_t *animation, framebuffer_t *fb)
{
    CHECK_ARG(animation && fb);

    animation->fb = fb;
    animation->timer = NULL;
    esp_timer_create_args_t timer_args = {
        .arg = animation,
        .callback = display_frame,
        .dispatch_method = ESP_TIMER_TASK
    };
    return esp_timer_create(&timer_args, &animation->timer);
}

esp_err_t fb_animation_play(fb_animation_t *animation, uint8_t fps, fb_draw_cb_t draw, void *render_ctx)
{
    CHECK_ARG(animation && fps && draw);

    animation->render_ctx = render_ctx;
    animation->draw = draw;
    return esp_timer_start_periodic(animation->timer, 1000000 / fps);
}

esp_err_t fb_animation_stop(fb_animation_t *animation)
{
    CHECK_ARG(animation);

    return esp_timer_stop(animation->timer);
}

esp_err_t fb_animation_free(fb_animation_t *animation)
{
    CHECK_ARG(animation);

    esp_timer_stop(animation->timer);
    return esp_timer_delete(animation->timer);
}

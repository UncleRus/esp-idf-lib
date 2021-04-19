#include <stdlib.h>
#include "led_effect.h"

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define BUF_SIZE(state) ((state)->width * (state)->height * sizeof(rgb_t))

esp_err_t led_effect_init(led_effect_t *state, size_t width, size_t height, led_effect_render_cb_t render_cb)
{
    CHECK_ARG(state && width && height && render_cb);

    state->width = width;
    state->height = height;
    state->frame_num = 0;
    state->last_frame_us = 0;
    state->render = render_cb;
    state->internal = NULL;
    state->frame_buf = calloc(1, BUF_SIZE(state));
    if (!state->frame_buf)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t led_effect_free(led_effect_t *state)
{
    CHECK_ARG(state);

    if (state->frame_buf)
        free(state->frame_buf);

    return ESP_OK;
}

esp_err_t led_effect_set_pixel_rgb(led_effect_t *state, size_t x, size_t y, rgb_t color)
{
    CHECK_ARG(state && state->frame_buf && x < state->width && y < state->height);

    uint8_t *pixel = state->frame_buf + LED_EFFECT_FRAMEBUF_OFFS(state, x, y);
    *((rgb_t *)pixel) = color;

    return ESP_OK;
}

esp_err_t led_effect_set_pixel_hsv(led_effect_t *state, size_t x, size_t y, hsv_t color)
{
    CHECK_ARG(state && state->frame_buf && x < state->width && y < state->height);

    uint8_t *pixel = state->frame_buf + LED_EFFECT_FRAMEBUF_OFFS(state, x, y);
    *((rgb_t *)pixel) = hsv2rgb_rainbow(color);

    return ESP_OK;
}

esp_err_t led_effect_get_pixel_rgb(led_effect_t *state, size_t x, size_t y, rgb_t *color)
{
    CHECK_ARG(color && state && state->frame_buf && x < state->width && y < state->height);

    uint8_t *pixel = state->frame_buf + LED_EFFECT_FRAMEBUF_OFFS(state, x, y);
    *color = *((rgb_t *)pixel);

    return ESP_OK;
}

esp_err_t led_effect_get_pixel_hsv(led_effect_t *state, size_t x, size_t y, hsv_t *color)
{
    CHECK_ARG(color && state && state->frame_buf && x < state->width && y < state->height);

    uint8_t *pixel = state->frame_buf + LED_EFFECT_FRAMEBUF_OFFS(state, x, y);
    *color = rgb2hsv_approximate(*((rgb_t *)pixel));

    return ESP_OK;
}

esp_err_t led_effect_clear(led_effect_t *state)
{
    CHECK_ARG(state && state->frame_buf);

    memset(state->frame_buf, 0, BUF_SIZE(state));

    return ESP_OK;
}

esp_err_t led_effect_begin_frame(led_effect_t *state)
{
    CHECK_ARG(state);

    if (state->busy)
        return ESP_ERR_INVALID_STATE;

    state->busy = true;

    return ESP_OK;
}

esp_err_t led_effect_end_frame(led_effect_t *state)
{
    CHECK_ARG(state);

    state->frame_num++;
    state->last_frame_us = esp_timer_get_time();
    state->busy = false;

    return ESP_OK;
}

esp_err_t led_effect_render(led_effect_t *state, void *arg)
{
    CHECK_ARG(state && state->frame_buf && state->render);

    if (state->busy)
        return ESP_ERR_INVALID_STATE;

    return state->render(state, arg);
}

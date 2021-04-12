#include "led_animation.h"

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t led_animation_init(led_animation_t *state, size_t width, size_t height)
{
    CHECK_ARG(state && width && height);

    state->width = width;
    state->height = height;
    state->frame_num = 0;
    state->frame_buf = calloc(width * height, sizeof(rgb_t));
    memset(state->buf, 0, sizeof(state->buf));

    return ESP_OK;
}

esp_err_t led_animation_free(led_animation_t *state)
{
    CHECK_ARG(state && state->frame_buf);

    free(state->frame_buf);

    return ESP_OK;
}

esp_err_t led_animation_set_pixel(led_animation_t *state, size_t x, size_t y, rgb_t color)
{
    CHECK_ARG(state && state->frame_buf && x < state->width && y < state->height);

    state->frame_buf[y * state->width + x] = color;

    return ESP_OK;
}

esp_err_t led_animation_get_pixel(led_animation_t *state, size_t x, size_t y, rgb_t *color)
{
    CHECK_ARG(color && state && state->frame_buf && x < state->width && y < state->height);

    *color = state->frame_buf[y * state->width + x];

    return ESP_OK;
}

esp_err_t led_animation_render(led_animation_t *state, led_strip_t *strip)
{
    CHECK_ARG(state && state->frame_buf && strip && strip->buf);

    for (size_t y = 0; y < state->height; y++)
        for (size_t x = 0; x < state->width; x++)
        {
            size_t idx = y * state->width + (y % 2 ? state->width - x - 1 : x);
            led_strip_set_pixel(strip, idx, state->frame_buf[y * state->width + x]);
        }

    return ESP_OK;
}

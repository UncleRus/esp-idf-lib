/**
 * @file rainbow1.c
 *
 * Simple rainbow effect
 *
 * Parameters:
 *     scale  Density of rainbows
 *            Suggested range 10-50.
 *     speed  Speed with which the rainbow shimmers
 *            Suggested range 1-50.
 */
#include <lib8tion.h>
#include <noise.h>
#include <stdlib.h>
#include <esp_timer.h>

#include "rainbow1.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct
{
    led_effect_rainbow1_direction_t direction;
    uint8_t scale;
    uint8_t speed;
} params_t;

esp_err_t led_effect_rainbow1_init(led_effect_t *state, led_effect_rainbow1_direction_t direction,
        uint8_t scale, uint8_t speed)
{
    CHECK_ARG(state);

    state->internal = calloc(1, sizeof(params_t));
    if (!state->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_rainbow1_set_params(state, direction, scale, speed);
}

esp_err_t led_effect_rainbow1_done(led_effect_t *state)
{
    CHECK_ARG(state);

    if (state->internal)
        free(state->internal);

    return ESP_OK;
}

esp_err_t led_effect_rainbow1_set_params(led_effect_t *state, led_effect_rainbow1_direction_t direction,
        uint8_t scale, uint8_t speed)
{
    CHECK_ARG(state && state->internal);

    params_t *params = (params_t *)state->internal;
    params->direction = direction;
    params->scale = scale;
    params->speed = speed;

    return ESP_OK;
}

esp_err_t led_effect_rainbow1_run(led_effect_t *state)
{
    CHECK_ARG(state);

    params_t *params = (params_t *)state->internal;

    if (params->direction == RAINBOW1_DIAGONAL)
    {
        for (size_t x = 0; x < state->width; x++)
            for (size_t y = 0; y < state->height; y++)
            {
                float twirl = 3.0f * params->scale / 100.0f;
                hsv_t color = {
                    .hue = state->frame_num * params->speed * 2 + (state->width / state->height * x + y * twirl) * params->scale,
                    .sat = 255,
                    .val = 255
                };
                led_effect_set_pixel_hsv(state, x, y, color);
            }
    }
    else
    {
        size_t outer = params->direction == RAINBOW1_HORIZONTAL ? state->width : state->height;
        size_t inner = params->direction == RAINBOW1_HORIZONTAL ? state->height : state->width;

        for (size_t i = 0; i < outer; i++)
        {
            hsv_t color = {
                .hue = state->frame_num * params->speed + i * params->scale,
                .sat = 255,
                .val = 255
            };
            for (size_t j = 0; j < inner; j++)
                if (params->direction == RAINBOW1_HORIZONTAL)
                    led_effect_set_pixel_hsv(state, i, j, color);
                else
                    led_effect_set_pixel_hsv(state, j, i, color);
        }
    }

    return led_effect_end_frame(state);
}

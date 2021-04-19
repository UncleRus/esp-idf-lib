/**
 * @file rays.c
 *
 * Colored rays effect, based on Yaroslaw Turbin code (https://vk.com/ldirko, https://www.reddit.com/user/ldirko/)
 * https://editor.soulmatelights.com/gallery/819-colored-bursts
 * Max supported framebuffer size 256x256
 * Supported RGB framebuffer only
 *
 * Parameters:
 *   speed    - Speed of rays movement, 0 - 50
 *   min_rays - Minimal rays count, 1 - 10
 *   max_rays - Maximal rays count, 10 - 20
 */
#include <lib8tion.h>
#include <stdlib.h>

#include "rays.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct
{
    uint8_t speed;
    uint8_t min_rays;
    uint8_t max_rays;
    uint8_t hue;
    uint8_t num_rays;
} params_t;

esp_err_t led_effect_rays_init(led_effect_t *state, uint8_t speed, uint8_t min_rays, uint8_t max_rays)
{
    CHECK_ARG(state);

    // allocate internal storage
    state->internal = calloc(1, sizeof(params_t));
    if (!state->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_rays_set_params(state, speed, min_rays, max_rays);
}

esp_err_t led_effect_rays_done(led_effect_t *state)
{
    CHECK_ARG(state && state->internal);

    // free internal storage
    if (state->internal)
        free(state->internal);

    return ESP_OK;
}

esp_err_t led_effect_rays_set_params(led_effect_t *state, uint8_t speed, uint8_t min_rays, uint8_t max_rays)
{
    CHECK_ARG(state && state->internal);

    params_t *params = (params_t *)state->internal;
    params->speed = speed;
    params->min_rays = min_rays;
    params->max_rays = max_rays;

    return ESP_OK;
}

static void fade(led_effect_t *state, uint8_t scale)
{
    for (size_t y = 0; y < state->height; y++)
        for (size_t x = 0; x < state->width; x++)
        {
            rgb_t c;
            led_effect_get_pixel_rgb(state, x, y, &c);
            led_effect_set_pixel_rgb(state, x, y, rgb_scale(c, scale));
        }
}

static void line(led_effect_t *state, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, rgb_t color)
{
    uint8_t xsteps = abs8(x1 - x2) + 1;
    uint8_t ysteps = abs8(y1 - y2) + 1;
    uint8_t steps = xsteps >= ysteps ? xsteps : ysteps;

    for (uint8_t i = 1; i <= steps; i++)
    {
        uint8_t dx = lerp8by8(x1, x2, i * 255 / steps);
        uint8_t dy = lerp8by8(y1, y2, i * 255 / steps);

        rgb_t pixel;
        led_effect_get_pixel_rgb(state, dx, dy, &pixel);
        led_effect_set_pixel_rgb(state, dx, dy, rgb_scale_video(rgb_add_rgb(pixel, color), i * 255 / steps));
    }
}

static size_t xy(void *ctx, size_t x, size_t y)
{
    led_effect_t *state = (led_effect_t *)ctx;
    return y * state->width + x;
}

esp_err_t led_effect_rays_run(led_effect_t *state)
{
    CHECK(led_effect_begin_frame(state));

    params_t *params = (params_t *)state->internal;

    // change number of rays
    if (params->max_rays > params->min_rays && state->frame_num % 10)
    {
        if (random8(2))
        {
            params->num_rays++;
            if (params->num_rays > params->max_rays)
                params->num_rays = params->max_rays;
        }
        else
        {
            params->num_rays--;
            if (params->num_rays < params->min_rays)
                params->num_rays = params->min_rays;
        }
    }

    params->hue++;
    fade(state, 40);
    for (uint8_t i = 0; i < params->num_rays; i++)
    {
        uint8_t x1 = beatsin8(4 + params->speed, 0, (state->width - 1), 0, 0);
        uint8_t x2 = beatsin8(2 + params->speed, 0, (state->width - 1), 0, 0);
        uint8_t y1 = beatsin8(8 + params->speed, 0, (state->height - 1), 0, i * 24);
        uint8_t y2 = beatsin8(10 + params->speed, 0, (state->height - 1), 0, i * 48 + 64);

        line(state, x1, x2, y1, y2, hsv2rgb_rainbow(hsv_from_values(i * 255 / params->num_rays + params->hue, 255, 255)));
    }
    blur2d((rgb_t *)state->frame_buf, state->width, state->height, 8, xy, state);

    return led_effect_end_frame(state);
}

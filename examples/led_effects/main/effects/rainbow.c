/**
 * @file rainbow.c
 *
 * Simple rainbow effect
 *
 * Parameters:
 *     - scale:  Density of rainbows. Suggested range 10-50.
 *     - speed:  Speed with which the rainbow shimmers. Suggested range 1-50.
 */
#include <lib8tion.h>
#include <noise.h>
#include <stdlib.h>

#include "effects/rainbow.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct
{
    led_effect_rainbow_direction_t direction;
    uint8_t scale;
    uint8_t speed;
} params_t;

esp_err_t led_effect_rainbow_init(framebuffer_t *fb, led_effect_rainbow_direction_t direction,
        uint8_t scale, uint8_t speed)
{
    CHECK_ARG(fb);

    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_rainbow_set_params(fb, direction, scale, speed);
}

esp_err_t led_effect_rainbow_done(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_rainbow_set_params(framebuffer_t *fb, led_effect_rainbow_direction_t direction,
        uint8_t scale, uint8_t speed)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    params->direction = direction;
    params->scale = scale;
    params->speed = speed;

    return ESP_OK;
}

esp_err_t led_effect_rainbow_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    if (params->direction == RAINBOW_DIAGONAL)
    {
        for (size_t x = 0; x < fb->width; x++)
            for (size_t y = 0; y < fb->height; y++)
            {
                float twirl = 3.0f * params->scale / 100.0f;
                hsv_t color = {
                    .hue = fb->frame_num * params->speed * 2 + (fb->width / fb->height * x + y * twirl) * params->scale,
                    .sat = 255,
                    .val = 255
                };
                fb_set_pixel_hsv(fb, x, y, color);
            }
    }
    else
    {
        size_t outer = params->direction == RAINBOW_HORIZONTAL ? fb->width : fb->height;
        size_t inner = params->direction == RAINBOW_HORIZONTAL ? fb->height : fb->width;

        for (size_t i = 0; i < outer; i++)
        {
            hsv_t color = {
                .hue = fb->frame_num * params->speed + i * params->scale,
                .sat = 255,
                .val = 255
            };
            for (size_t j = 0; j < inner; j++)
                if (params->direction == RAINBOW_HORIZONTAL)
                    fb_set_pixel_hsv(fb, i, j, color);
                else
                    fb_set_pixel_hsv(fb, j, i, color);
        }
    }

    return fb_end(fb);
}

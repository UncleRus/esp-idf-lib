/**
 * @file waterfall.c
 *
 * Waterfall/Fire effect
 *
 * Parameters:
 *     - hue:      Basic hue for waterfall palette. Ignored when mode = WATERFALL_FIRE
 *     - cooling:  How much does the air cool as it rises. Less cooling = taller flames,
 *                 more cooling = shorter flames. Suggested range 20-100.
 *     - sparking: Chance (out of 255) that a new spark will light up. Suggested range 50-200.
 *
 * Recommended parameters for fire mode: cooling = 90, sparking = 80
 */
#include <lib8tion.h>
#include <stdlib.h>

#include "effects/waterfall.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define PALETTE_SIZE 16

typedef struct
{
    led_effect_waterfall_mode_t mode;
    uint8_t hue;
    uint8_t cooling;
    uint8_t sparking;
    rgb_t palette[PALETTE_SIZE];
    uint8_t *map;
} params_t;

esp_err_t led_effect_waterfall_init(framebuffer_t *fb, led_effect_waterfall_mode_t mode,
        uint8_t hue, uint8_t cooling, uint8_t sparking)
{
    CHECK_ARG(fb);

    // allocate internal storage
    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    // allocate color map
    params_t *params = (params_t *)fb->internal;
    params->map = calloc(fb->width * fb->height, 1);
    if (!params->map)
        return ESP_ERR_NO_MEM;

    return led_effect_waterfall_set_params(fb, mode, hue, cooling, sparking);
}

esp_err_t led_effect_waterfall_done(framebuffer_t *fb)
{
    CHECK_ARG(fb);
    params_t *params = (params_t *)fb->internal;

    // free map
    if (params && params->map)
        free(params->map);

    // free internal storage
    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_waterfall_set_params(framebuffer_t *fb, led_effect_waterfall_mode_t mode,
        uint8_t hue, uint8_t cooling, uint8_t sparking)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    params->mode = mode;
    params->hue = hue;
    params->cooling = cooling;
    params->sparking = sparking;
    switch (mode)
    {
        case WATERFALL_SIMPLE:
            rgb_fill_gradient4_hsv(params->palette, PALETTE_SIZE,
                    hsv_from_values(0, 0, 0),
                    hsv_from_values(hue, 0, 255),
                    hsv_from_values(hue, 128, 255),
                    hsv_from_values(hue, 255, 255),
                    COLOR_SHORTEST_HUES);
            break;
        case WATERFALL_COLORS:
            rgb_fill_gradient4_hsv(params->palette, PALETTE_SIZE,
                    hsv_from_values(0, 0, 0),
                    hsv_from_values(hue, 0, 255),
                    hsv_from_values(hue, 128, 255),
                    hsv_from_values(255, 255, 255),
                    COLOR_SHORTEST_HUES);
            break;
        case WATERFALL_FIRE:
            rgb_fill_gradient4_rgb(params->palette, PALETTE_SIZE,
                    rgb_from_values(0, 0, 0),       // black
                    rgb_from_values(255, 0, 0),
                    rgb_from_values(255, 255, 0),
                    rgb_from_values(255, 255, 255)); // white
            break;
        case WATERFALL_COLD_FIRE:
            rgb_fill_gradient4_rgb(params->palette, PALETTE_SIZE,
                    rgb_from_values(0, 0, 0),       // black
                    rgb_from_values(0, 0, 100),
                    rgb_from_values(0, 200, 255),
                    rgb_from_values(255, 255, 255)); // white
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

#define MAP_XY(x, y) ((y) * fb->width + (x))

esp_err_t led_effect_waterfall_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    for (size_t x = 0; x < fb->width; x++)
    {
        size_t y;

        // Step 1.  Cool down every cell a little
        for (y = 0; y < fb->height; y++)
            params->map[MAP_XY(x, y)] = qsub8(params->map[MAP_XY(x, y)], random8_to((params->cooling * 10 / fb->height) + 2));

        // Step 2.  Heat from each cell drifts 'up' and diffuses a little
        for (y = fb->height - 1; y >= 2; y--)
            params->map[MAP_XY(x, y)] =
                    (params->map[MAP_XY(x, y - 1)] + params->map[MAP_XY(x, y - 2)] + params->map[MAP_XY(x, y - 2)]) / 3;

        // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
        if (random8() < params->sparking)
        {
            y = random8_to(2);
            params->map[MAP_XY(x, y)] = qadd8(params->map[MAP_XY(x, y)], random8_between(160, 255));
        }

        // Step 4.  Map from heat cells to LED colors
        for (y = 0; y < fb->height; y++)
        {
            // Scale the heat value from 0-255 down to 0-240
            // for best results with color palettes.
            uint8_t color_idx = scale8(params->map[MAP_XY(x, y)], 240);
            bool is_fire = (params->mode == WATERFALL_FIRE || params->mode == WATERFALL_COLD_FIRE);
            fb_set_pixel_rgb(fb, x, is_fire ? y : fb->height - 1 - y,
                    color_from_palette_rgb(params->palette, PALETTE_SIZE, color_idx, 255, true));
        }
    }

    return fb_end(fb);
}

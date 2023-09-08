/**
 * @file rain.c
 *
 * Rain effect by Shaitan
 *
 */
#include <lib8tion.h>
#include <stdlib.h>

#include "effects/rain.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct
{
    led_effect_rain_mode_t mode;
    uint8_t hue;
    uint8_t density;
    uint8_t tail;
} params_t;

esp_err_t led_effect_rain_init(framebuffer_t *fb, led_effect_rain_mode_t mode, uint8_t hue, uint8_t density, uint8_t tail)
{
    CHECK_ARG(fb);

    // allocate internal storage
    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_rain_set_params(fb, mode, hue, density, tail);
}

esp_err_t led_effect_rain_done(framebuffer_t *fb)
{
    CHECK_ARG(fb && fb->internal);

    // free internal storage
    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_rain_set_params(framebuffer_t *fb, led_effect_rain_mode_t mode, uint8_t hue, uint8_t density, uint8_t tail)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    params->mode = mode;
    params->hue = hue;
    params->density = 255 - qadd8(density, 155);
    params->tail = tail;

    return ESP_OK;
}

esp_err_t led_effect_rain_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    for (size_t x = 0; x < fb->width; x++)
    {
        rgb_t c;
        fb_get_pixel_rgb(fb, x, fb->height - 1, &c);
        if (!rgb_luma(c) && random8_to(params->density) == 0)
            fb_set_pixel_hsv(fb, x, fb->height - 1,
                    hsv_from_values(params->mode == RAIN_MODE_SINGLE_COLOR ? params->hue : random8(), 255, 255));
        else
        {
            c = rgb_scale(c, params->tail + random8_to(100) - 50);
            fb_set_pixel_rgb(fb, x, fb->height - 1, rgb_luma(c) < 3 ? rgb_from_values(0, 0, 0) : c);
        }
    }
    fb_shift(fb, 1, FB_SHIFT_DOWN);

    return fb_end(fb);
}

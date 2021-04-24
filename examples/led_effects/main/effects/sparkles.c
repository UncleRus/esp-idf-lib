/**
 * @file sparkles.c
 *
 * Colored sparkles effect
 *
 */
#include <lib8tion.h>
#include <stdlib.h>

#include "effects/sparkles.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct
{
    uint8_t max_sparkles;
    uint8_t fadeout_speed;
} params_t;

esp_err_t led_effect_sparkles_init(framebuffer_t *fb, uint8_t max_sparkles, uint8_t fadeout_speed)
{
    CHECK_ARG(fb);

    // allocate internal storage
    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_sparkles_set_params(fb, max_sparkles, fadeout_speed);
}

esp_err_t led_effect_sparkles_done(framebuffer_t *fb)
{
    CHECK_ARG(fb && fb->internal);

    // free internal storage
    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_sparkles_set_params(framebuffer_t *fb, uint8_t max_sparkles, uint8_t fadeout_speed)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    params->max_sparkles = max_sparkles;
    params->fadeout_speed = fadeout_speed;

    return ESP_OK;
}

esp_err_t led_effect_sparkles_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    fb_blur2d(fb, 8);
    for (uint8_t i = 0; i < params->max_sparkles; i++)
    {
        uint16_t x = random16_to(fb->width);
        uint16_t y = random16_to(fb->height);

        rgb_t c;
        fb_get_pixel_rgb(fb, x, y, &c);
        if (rgb_luma(c) < 5)
            fb_set_pixel_hsv(fb, x, y, hsv_from_values(random8(), 255, 255));
    }
    fb_fade(fb, params->fadeout_speed);

    return fb_end(fb);
}

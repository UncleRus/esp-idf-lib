/**
 * @file noise.c
 *
 * Perlin noise effect
 *
 * Author: Chuck Sommerville
 */
#include <lib8tion.h>
#include <noise.h>
#include <stdlib.h>

#include "noise.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct
{
    uint8_t scale;
    uint8_t speed;
    uint16_t z_pos;
    uint16_t x_offs;
    uint8_t hue;
} params_t;

esp_err_t led_effect_noise_init(framebuffer_t *fb, uint8_t scale, uint8_t speed)
{
    CHECK_ARG(fb);

    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_noise_set_params(fb, scale, speed);
}

esp_err_t led_effect_noise_done(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_noise_set_params(framebuffer_t *fb, uint8_t scale, uint8_t speed)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    params->scale = scale;
    params->speed = speed;

    return ESP_OK;
}

esp_err_t led_effect_noise_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    if (!(fb->frame_num % 30))
        params->x_offs++;

    params->z_pos += params->speed;
    params->hue++;

    for (int x = 0; x < fb->width; x++)
        for (int y = 0; y < fb->height; y++)
        {
            uint8_t noise = inoise8_3d(x * params->scale, y * params->scale, params->z_pos);
            fb_set_pixel_hsv(fb, x, y, hsv_from_values(params->hue + noise, 255, 255));
        }

    return fb_end(fb);
}

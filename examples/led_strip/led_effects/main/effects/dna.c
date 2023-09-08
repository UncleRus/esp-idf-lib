/**
 * @file dna.c
 *
 * DNA spiral effect
 *
 * Author: Yaroslaw Turbin (https://vk.com/ldirko, https://www.reddit.com/user/ldirko/)
 *
 * Max supported framebuffer size is 256x256
 *
 * Parameters:
 *   speed  - Speed of rotation, 10 - 100
 *   size   - Spiral size, 1 - 10
 *   border - Add white border
 */
#include <lib8tion.h>
#include <stdlib.h>

#include "effects/dna.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define PALETTE_SIZE 16

typedef struct
{
    uint8_t speed;
    uint8_t size;
    bool border;
    uint32_t offset;
} params_t;

esp_err_t led_effect_dna_init(framebuffer_t *fb, uint8_t speed, uint8_t size, bool border)
{
    CHECK_ARG(fb);

    // allocate internal storage
    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_dna_set_params(fb, speed, size, border);
}

esp_err_t led_effect_dna_done(framebuffer_t *fb)
{
    CHECK_ARG(fb && fb->internal);

    // free internal storage
    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_dna_set_params(framebuffer_t *fb, uint8_t speed, uint8_t size, bool border)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    params->speed = speed;
    params->size = size;
    params->border = border;

    return ESP_OK;
}

static const rgb_t dark_slate_gray = { .r = 0x2f, .g = 0x4f, .b = 0x4f };
static const rgb_t white = { .r = 0xff, .g = 0xff, .b = 0xff };

void horizontal_line(framebuffer_t *fb, uint8_t x1, uint8_t x2, uint8_t y, rgb_t color, bool dot)
{
    uint8_t steps = abs8(x2 - x1) + 1;

    for (uint8_t i = 1; i <= steps; i++)
    {
        uint8_t dx = lerp8by8(x1, x2, i * 255 / steps);
        rgb_t pixel;
        fb_get_pixel_rgb(fb, dx, y, &pixel);
        pixel = rgb_scale_video(rgb_add_rgb(pixel, color), i * 255 / steps);
        fb_set_pixel_rgb(fb, dx, y, pixel);
    }

    if (dot)
    {
        //add white point at the ends of line
        rgb_t pixel;
        fb_get_pixel_rgb(fb, x1, y, &pixel);
        fb_set_pixel_rgb(fb, x1, y, rgb_add_rgb(pixel, dark_slate_gray));
        fb_set_pixel_rgb(fb, x2, y, white);
    }
}

esp_err_t led_effect_dna_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    params->offset += params->speed / 10;

    fb_fade(fb, 130);

    for (uint8_t i = 0; i < fb->height; i++)
    {
        uint16_t x1 = beatsin8(params->speed, 0, fb->width - 1, 0, i * params->size) + beatsin8(params->speed - 7, 0, fb->width - 1, 0, i * params->size + 128);
        uint16_t x2 = beatsin8(params->speed, 0, fb->width - 1, 0, 128 + i * params->size) + beatsin8(params->speed - 7, 0, fb->width - 1, 0, 128 + 64 + i * params->size);

        rgb_t color = hsv2rgb_rainbow(hsv_from_values(i * 128 / (fb->height - 1) + params->offset, 255, 255));

        if ((i + params->offset / 8) & 3)
            horizontal_line(fb, x1 / 2, x2 / 2, i, color, params->border);
    }

    return fb_end(fb);
}

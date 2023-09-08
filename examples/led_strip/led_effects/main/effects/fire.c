/**
 * @file fire.c
 *
 * Fire effect based on Perlin noise
 *
 * Author: Yaroslaw Turbin (https://vk.com/ldirko, https://www.reddit.com/user/ldirko/)
 *
 * https://pastebin.com/jSSVSRi6
 */
#include <lib8tion.h>
#include <noise.h>
#include <stdlib.h>

#include "effects/fire.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define PALETTE_SIZE 16

typedef struct
{
    rgb_t palette[PALETTE_SIZE];
} params_t;

esp_err_t led_effect_fire_init(framebuffer_t *fb, led_effect_fire_palette_t p)
{
    CHECK_ARG(fb);

    // allocate internal storage
    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_fire_set_params(fb, p);
}

static const rgb_t C_BLACK  = { 0 };
static const rgb_t C_WHITE  = { .r = 255, .g = 255, .b = 255 };
static const rgb_t C_DBLUE  = { .r = 0,   .g = 0,   .b = 100 };
static const rgb_t C_CYAN   = { .r = 0,   .g = 200, .b = 255 };
static const rgb_t C_RED    = { .r = 255, .g = 0,   .b = 0 };
static const rgb_t C_YELLOW = { .r = 255, .g = 255, .b = 0 };
static const rgb_t C_DGREEN = { .r = 0,   .g = 100, .b = 0 };
static const rgb_t C_BGREEN = { .r = 155, .g = 255, .b = 155 };

esp_err_t led_effect_fire_set_params(framebuffer_t *fb, led_effect_fire_palette_t p)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    switch (p)
    {
        case FIRE_PALETTE_BLUE:
            rgb_fill_gradient4_rgb(params->palette, PALETTE_SIZE, C_BLACK, C_DBLUE, C_CYAN, C_WHITE);
            break;
        case FIRE_PALETTE_GREEN:
            rgb_fill_gradient4_rgb(params->palette, PALETTE_SIZE, C_BLACK, C_DGREEN, C_BGREEN, C_WHITE);
            break;
        default:
            rgb_fill_gradient4_rgb(params->palette, PALETTE_SIZE, C_BLACK, C_RED, C_YELLOW, C_WHITE);
    }

    return ESP_OK;
}

esp_err_t led_effect_fire_done(framebuffer_t *fb)
{
    CHECK_ARG(fb && fb->internal);

    // free internal storage
    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_fire_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    uint32_t a = esp_timer_get_time() / 1000;

    for (size_t x = 0; x < fb->width; x++)
        for (size_t y = 0; y < fb->height; y++)
        {
            uint8_t idx = qsub8(inoise8_3d(x * 60, y * 60 + a, a / 3), abs8(y - (fb->height - 1)) * 255 / (fb->height - 1));
            rgb_t c = color_from_palette_rgb(params->palette, PALETTE_SIZE, idx, 255, true);
            fb_set_pixel_rgb(fb, x, fb->height - y - 1, c);
        }

    return fb_end(fb);
}


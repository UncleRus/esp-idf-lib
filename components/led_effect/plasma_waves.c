/**
 * @file plasma_waves.c
 *
 * Plasma waves effect.
 * Author: Edmund "Skorn" Horn
 */
#include <lib8tion.h>
#include <stdlib.h>
#include "effects/plasma_waves.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const uint8_t exp_gamma[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
    1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,
    4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,   6,   6,   7,   7,
    7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,  11,  11,  12,  12,
    12,  13,  13,  14,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,
    19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  26,  26,  27,  28,
    28,  29,  30,  30,  31,  32,  32,  33,  34,  35,  35,  36,  37,  38,  39,
    39,  40,  41,  42,  43,  44,  44,  45,  46,  47,  48,  49,  50,  51,  52,
    53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,
    68,  70,  71,  72,  73,  74,  75,  77,  78,  79,  80,  82,  83,  84,  85,
    87,  89,  91,  92,  93,  95,  96,  98,  99,  100, 101, 102, 105, 106, 108,
    109, 111, 112, 114, 115, 117, 118, 120, 121, 123, 125, 126, 128, 130, 131,
    133, 135, 136, 138, 140, 142, 143, 145, 147, 149, 151, 152, 154, 156, 158,
    160, 162, 164, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187,
    190, 192, 194, 196, 198, 200, 202, 204, 207, 209, 211, 213, 216, 218, 220,
    222, 225, 227, 229, 232, 234, 236, 239, 241, 244, 246, 249, 251, 253, 254,
    255
};

typedef struct
{
    uint8_t speed;
} params_t;

esp_err_t led_effect_plasma_waves_init(framebuffer_t *fb, uint8_t speed)
{
    CHECK_ARG(fb);

    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_plasma_waves_set_params(fb, speed);
}

esp_err_t led_effect_plasma_waves_done(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_plasma_waves_set_params(framebuffer_t *fb, uint8_t speed)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    params->speed = scale8_video(256 - speed, 150);
    if (!params->speed) params->speed = 1;

    return ESP_OK;
}

esp_err_t led_effect_plasma_waves_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    uint8_t t1 = cos8((42 * fb->frame_num) / params->speed);
    uint8_t t2 = cos8((35 * fb->frame_num) / params->speed);
    uint8_t t3 = cos8((38 * fb->frame_num) / params->speed);

    for (uint16_t y = 0; y < fb->height; y++)
    {
        for (uint16_t x = 0; x < fb->width; x++)
        {
            // Calculate 3 separate plasma waves, one for each color channel
            uint8_t r = cos8((x << 3) + (t1 >> 1) + cos8(t2 + (y << 3)));
            uint8_t g = cos8((y << 3) + t1 + cos8((t3 >> 2) + (x << 3)));
            uint8_t b = cos8((y << 3) + t2 + cos8(t1 + x + (g >> 2)));

            rgb_t c = {
                .r = exp_gamma[r],
                .g = exp_gamma[g],
                .b = exp_gamma[b]
            };
            fb_set_pixel_rgb(fb, x, y, c);
        }
    }

    return fb_end(fb);
}

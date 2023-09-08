/**
 * @file matrix.c
 *
 * Matrix effect
 *
 */
#include <lib8tion.h>
#include <stdlib.h>

#include "effects/matrix.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct
{
    uint8_t density;
} params_t;

esp_err_t led_effect_matrix_init(framebuffer_t *fb, uint8_t density)
{
    CHECK_ARG(fb);

    // allocate internal storage
    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_matrix_set_params(fb, density);
}

esp_err_t led_effect_matrix_done(framebuffer_t *fb)
{
    CHECK_ARG(fb && fb->internal);

    // free internal storage
    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

esp_err_t led_effect_matrix_set_params(framebuffer_t *fb, uint8_t density)
{
    CHECK_ARG(fb && fb->internal);

    params_t *params = (params_t *)fb->internal;
    params->density = 255 - density;

    return ESP_OK;
}

#define MATRIX_START_COLOR   0x9bf800
#define MATRIX_DIM_COLOR     0x558800
#define MATRIX_STEP          0x0a1000
#define MATRIX_ALMOST_OFF    0x050800
#define MATRIX_OFF_THRESH    0x030000
#define MATRIX_DIMMEST_COLOR 0x020300

esp_err_t led_effect_matrix_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    for (size_t x = 0; x < fb->width; x++)
    {
        // process matrix from bottom to the second line from the top
        for (size_t y = 0; y < fb->height - 1; y++)
        {
            rgb_t cur_color, upper_color;

            // get current pixel color
            fb_get_pixel_rgb(fb, x, y, &cur_color);
            uint32_t cur_code = rgb_to_code(cur_color);
            // get color of the pixel above current
            fb_get_pixel_rgb(fb, x, y + 1, &upper_color);
            uint32_t upper_code = rgb_to_code(upper_color);

            // if above is max brightness, ignore this fact with some probability or move tail down
            if (upper_code == MATRIX_START_COLOR && random8_to(7 * fb->height) != 0)
                fb_set_pixel_rgb(fb, x, y, upper_color);
            // if current pixel is off, light up new tails with some probability
            else if (cur_code == 0 && random8_to(params->density) == 0)
                fb_set_pixel_rgb(fb, x, y, rgb_from_code(MATRIX_START_COLOR));
            // if current pixel is almost off, try to make the fading out slower
            else if (cur_code <= MATRIX_ALMOST_OFF)
            {
                if (cur_code >= MATRIX_OFF_THRESH)
                    fb_set_pixel_rgb(fb, x, y, rgb_from_code(MATRIX_DIMMEST_COLOR));
                else if (cur_code != 0)
                    fb_set_pixel_rgb(fb, x, y, rgb_from_code(0));
            }
            else if (cur_code == MATRIX_START_COLOR)
                // first step of tail fading
                fb_set_pixel_rgb(fb, x, y, rgb_from_code(MATRIX_DIM_COLOR));
            else
                // otherwise just lower the brightness one step
                fb_set_pixel_rgb(fb, x, y, rgb_from_code(cur_code - MATRIX_STEP));
        }

        // upper line processing
        rgb_t cur_color;
        fb_get_pixel_rgb(fb, x, fb->height - 1, &cur_color);
        uint32_t cur_code = rgb_to_code(cur_color);

        // if current top pixel is off, fill it with some probability
        if (cur_code == 0)
        {
            if (random8_to(params->density) == 0)
                fb_set_pixel_rgb(fb, x, fb->height - 1, rgb_from_code(MATRIX_START_COLOR));
        }
        // if current pixel is almost off, try to make the fading out slower
        else if (cur_code <= MATRIX_ALMOST_OFF)
        {
            if (cur_code >= MATRIX_OFF_THRESH)
                fb_set_pixel_rgb(fb, x, fb->height - 1, rgb_from_code(MATRIX_DIMMEST_COLOR));
            else
                fb_set_pixel_rgb(fb, x, fb->height - 1, rgb_from_code(0));
        }
        else if (cur_code == MATRIX_START_COLOR)
            // first step of tail fading
            fb_set_pixel_rgb(fb, x, fb->height - 1, rgb_from_code(MATRIX_DIM_COLOR));
        else
            // otherwise just lower the brightness one step
            fb_set_pixel_rgb(fb, x, fb->height - 1, rgb_from_code(cur_code - MATRIX_STEP));
    }

    return fb_end(fb);
}

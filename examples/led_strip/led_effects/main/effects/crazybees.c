/**
 * @file crazybees.c
 *
 * Crazy Bees effect
 *
 * Author: Stepko
 */
#include <lib8tion.h>
#include <stdlib.h>

#include "effects/crazybees.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct
{
    size_t x, y;
    size_t flower_x, flower_y;
    uint8_t hue;
} bee_t;

typedef struct
{
    uint8_t num_bees;
    bee_t bees[CRAZYBEES_MAX_BEES];
} params_t;

esp_err_t led_effect_crazybees_init(framebuffer_t *fb, uint8_t num_bees)
{
    CHECK_ARG(fb && num_bees && num_bees <= CRAZYBEES_MAX_BEES);

    fb->internal = calloc(1, sizeof(params_t));
    if (!fb->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_crazybees_set_params(fb, num_bees);
}

esp_err_t led_effect_crazybees_done(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    if (fb->internal)
        free(fb->internal);

    return ESP_OK;
}

static void change_flower(framebuffer_t *fb, uint8_t bee)
{
    params_t *params = (params_t *)fb->internal;
    params->bees[bee].flower_x = random8_to(fb->width);
    params->bees[bee].flower_y = random8_to(fb->height);
    params->bees[bee].hue = random8();
}

esp_err_t led_effect_crazybees_set_params(framebuffer_t *fb, uint8_t num_bees)
{
    CHECK_ARG(fb && fb->internal && num_bees && num_bees <= CRAZYBEES_MAX_BEES);

    params_t *params = (params_t *)fb->internal;
    params->num_bees = num_bees;

    for (uint8_t i = 0; i < num_bees; i++)
    {
        // set bee
        params->bees[i].x = random8_to(fb->width);
        params->bees[i].y = random8_to(fb->height);
        // set flower
        change_flower(fb, i);
    }

    return ESP_OK;
}

esp_err_t led_effect_crazybees_run(framebuffer_t *fb)
{
    CHECK(fb_begin(fb));

    params_t *params = (params_t *)fb->internal;

    static rgb_t white = { .r = 255, .g = 255, .b = 255 };

    fb_fade(fb, 8);
    for (uint8_t i = 0; i < params->num_bees; i++)
    {
        // move bee
        if (params->bees[i].x > params->bees[i].flower_x) params->bees[i].x--;
        if (params->bees[i].y > params->bees[i].flower_y) params->bees[i].y--;
        if (params->bees[i].x < params->bees[i].flower_x) params->bees[i].x++;
        if (params->bees[i].y < params->bees[i].flower_y) params->bees[i].y++;

        // bingo, change flower
        if (params->bees[i].x == params->bees[i].flower_x && params->bees[i].y == params->bees[i].flower_y)
            change_flower(fb, i);

        // draw bee
        fb_set_pixel_rgb(fb, params->bees[i].x, params->bees[i].y, white);

        // draw flower
        hsv_t c = { .h = params->bees[i].hue, .s = 255, .v = 255 };
        fb_set_pixel_hsv(fb, params->bees[i].flower_x - 1, params->bees[i].flower_y, c);
        fb_set_pixel_hsv(fb, params->bees[i].flower_x, params->bees[i].flower_y - 1, c);
        fb_set_pixel_hsv(fb, params->bees[i].flower_x + 1, params->bees[i].flower_y, c);
        fb_set_pixel_hsv(fb, params->bees[i].flower_x, params->bees[i].flower_y + 1, c);
    }
    fb_blur2d(fb, 16);

    return fb_end(fb);
}

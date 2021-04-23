/**
 * @file crazybees.c
 *
 * Crazy Bees effect
 * Author: stepko
 */
#include <lib8tion.h>
#include <noise.h>
#include <stdlib.h>
#include <esp_timer.h>

#include "crazybees.h"

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

esp_err_t led_effect_crazybees_init(led_effect_t *state, uint8_t num_bees)
{
    CHECK_ARG(state && num_bees && num_bees <= CRAZYBEES_MAX_BEES);

    state->internal = calloc(1, sizeof(params_t));
    if (!state->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_crazybees_set_params(state, num_bees);
}

esp_err_t led_effect_crazybees_done(led_effect_t *state)
{
    CHECK_ARG(state);

    if (state->internal)
        free(state->internal);

    return ESP_OK;
}

static void change_flower(led_effect_t *state, uint8_t bee)
{
    params_t *params = (params_t *)state->internal;
    params->bees[bee].flower_x = random8_to(state->width);
    params->bees[bee].flower_y = random8_to(state->height);
    params->bees[bee].hue = random8();
}

esp_err_t led_effect_crazybees_set_params(led_effect_t *state, uint8_t num_bees)
{
    CHECK_ARG(state && state->internal && num_bees && num_bees <= CRAZYBEES_MAX_BEES);

    params_t *params = (params_t *)state->internal;
    params->num_bees = num_bees;

    for (uint8_t i = 0; i < num_bees; i++)
    {
        // set bee
        params->bees[i].x = random8_to(state->width);
        params->bees[i].y = random8_to(state->height);
        // set flower
        change_flower(state, i);
    }

    return ESP_OK;
}

static size_t xy(void *ctx, size_t x, size_t y)
{
    led_effect_t *state = (led_effect_t *)ctx;
    return y * state->width + x;
}

esp_err_t led_effect_crazybees_run(led_effect_t *state)
{
    CHECK(led_effect_begin_frame(state));

    params_t *params = (params_t *)state->internal;

    static rgb_t white = { .r = 255, .g = 255, .b = 255 };

    led_effect_fade(state, 8);
    for (uint8_t i = 0; i < params->num_bees; i++)
    {
        // move bee
        if (params->bees[i].x > params->bees[i].flower_x) params->bees[i].x--;
        if (params->bees[i].y > params->bees[i].flower_y) params->bees[i].y--;
        if (params->bees[i].x < params->bees[i].flower_x) params->bees[i].x++;
        if (params->bees[i].y < params->bees[i].flower_y) params->bees[i].y++;

        // bingo, change flower
        if (params->bees[i].x == params->bees[i].flower_x && params->bees[i].y == params->bees[i].flower_y)
            change_flower(state, i);

        // draw bee
        led_effect_set_pixel_rgb(state, params->bees[i].x, params->bees[i].y, white);

        // draw flower
        hsv_t c = { .h = params->bees[i].hue, .s = 255, .v = 255 };
        led_effect_set_pixel_hsv(state, params->bees[i].flower_x - 1, params->bees[i].flower_y, c);
        led_effect_set_pixel_hsv(state, params->bees[i].flower_x, params->bees[i].flower_y - 1, c);
        led_effect_set_pixel_hsv(state, params->bees[i].flower_x + 1, params->bees[i].flower_y, c);
        led_effect_set_pixel_hsv(state, params->bees[i].flower_x, params->bees[i].flower_y + 1, c);
    }

    blur2d(state->frame_buf, state->width, state->height, 32, xy, state);

    return led_effect_end_frame(state);
}

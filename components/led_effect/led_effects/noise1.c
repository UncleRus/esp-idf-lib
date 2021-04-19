#include <lib8tion.h>
#include <noise.h>
#include <stdlib.h>
#include <esp_timer.h>

#include "noise1.h"

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

esp_err_t led_effect_noise1_init(led_effect_t *state, uint8_t scale, uint8_t speed)
{
    CHECK_ARG(state);

    state->internal = calloc(1, sizeof(params_t));
    if (!state->internal)
        return ESP_ERR_NO_MEM;

    return led_effect_noise1_set_params(state, scale, speed);
}

esp_err_t led_effect_noise1_done(led_effect_t *state)
{
    CHECK_ARG(state);

    if (state->internal)
        free(state->internal);

    return ESP_OK;
}

esp_err_t led_effect_noise1_set_params(led_effect_t *state, uint8_t scale, uint8_t speed)
{
    CHECK_ARG(state && state->internal);

    params_t *params = (params_t *)state->internal;
    params->scale = scale;
    params->speed = speed;

    return ESP_OK;
}

esp_err_t led_effect_noise1_run(led_effect_t *state)
{
    CHECK_ARG(state);

    params_t *params = (params_t *)state->internal;

    uint32_t frame_timeout = (esp_timer_get_time() - state->last_frame_us) / 1000;
    if (frame_timeout >= 1000)
        params->x_offs++;
    if (frame_timeout >= 30)
    {
        params->z_pos += params->speed;
        params->hue++;
    }

    for (int x = 0; x < state->width; x++)
        for (int y = 0; y < state->height; y++)
        {
            uint8_t noise = inoise8_3d(x * params->scale, y * params->scale, params->z_pos);
            led_effect_set_pixel_hsv(state, x, y, hsv_from_values(params->hue + noise, 255, 255));
        }

    return led_effect_end_frame(state);
}

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>
#include "led_animation.h"

#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO 5
#define LED_CHANNEL RMT_CHANNEL_0
#define LED_MATRIX_WIDTH  16
#define LED_MATRIX_HEIGHT 16
#define LED_BRIGHTNESS 30 // 0..255

////////////////////////////////////////////////////////////////////////////////

#define MATRIX_START_COLOR   0x9bf800
#define MATRIX_DIM_COLOR     0x558800
#define MATRIX_STEP          0x0a1000
#define MATRIX_ALMOST_OFF    0x050800
#define MATRIX_OFF_THRESH    0x030000
#define MATRIX_DIMMEST_COLOR 0x020300

void led_effect_matrix(led_animation_t *state, size_t scale)
{
    state->frame_num++;

    for (size_t x = 0; x < state->width; x++)
    {
        // process matrix from bottom to the second line from the top
        for (size_t y = 0; y < state->height - 1; y++)
        {
            rgb_t cur_color, upper_color;

            // get current pixel color
            led_animation_get_pixel(state, x, y, &cur_color);
            uint32_t cur_code = rgb_to_code(cur_color);
            // get color of the pixel above current
            led_animation_get_pixel(state, x, y + 1, &upper_color);
            uint32_t upper_code = rgb_to_code(upper_color);

            // if above is max brightness, ignore this fact with some probability or move tail down
            if (upper_code == MATRIX_START_COLOR && random8_to(7 * state->height) != 0)
                led_animation_set_pixel(state, x, y, upper_color);
            // if current pixel is off, light up new tails with some probability
            else if (cur_code == 0 && random8_to((100 - scale) * state->height) == 0)
                led_animation_set_pixel(state, x, y, rgb_from_code(MATRIX_START_COLOR));
            // if current pixel is almost off, try to make the fading out slower
            else if (cur_code <= MATRIX_ALMOST_OFF)
            {
                if (cur_code >= MATRIX_OFF_THRESH)
                    led_animation_set_pixel(state, x, y, rgb_from_code(MATRIX_DIMMEST_COLOR));
                else if (cur_code != 0)
                    led_animation_set_pixel(state, x, y, rgb_from_code(0));
            }
            else if (cur_code == MATRIX_START_COLOR)
                // first step of tail fading
                led_animation_set_pixel(state, x, y, rgb_from_code(MATRIX_DIM_COLOR));
            else
                // otherwise just lower the brightness one step
                led_animation_set_pixel(state, x, y, rgb_from_code(cur_code - MATRIX_STEP));
        }

        // upper line processing
        rgb_t cur_color;
        led_animation_get_pixel(state, x, state->height - 1, &cur_color);
        uint32_t cur_code = rgb_to_code(cur_color);

        // if current top pixel is off, fill it with some probability
        if (cur_code == 0)
        {
            if (random8_to(100 - scale) == 0)
                led_animation_set_pixel(state, x, state->height - 1, rgb_from_code(MATRIX_START_COLOR));
        }
        // if current pixel is almost off, try to make the fading out slower
        else if (cur_code <= MATRIX_ALMOST_OFF)
        {
            if (cur_code >= MATRIX_OFF_THRESH)
                led_animation_set_pixel(state, x, state->height - 1, rgb_from_code(MATRIX_DIMMEST_COLOR));
            else
                led_animation_set_pixel(state, x, state->height - 1, rgb_from_code(0));
        }
        else if (cur_code == MATRIX_START_COLOR)
            // first step of tail fading
            led_animation_set_pixel(state, x, state->height - 1, rgb_from_code(MATRIX_DIM_COLOR));
        else
            // otherwise just lower the brightness one step
            led_animation_set_pixel(state, x, state->height - 1, rgb_from_code(cur_code - MATRIX_STEP));
    }
}

////////////////////////////////////////////////////////////////////////////////

void test(void *pvParameters)
{
    led_strip_t strip = {
        .type = LED_TYPE,
        .length = LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT,
        .gpio = LED_GPIO,
        .channel = LED_CHANNEL,
        .buf = NULL,
#ifdef LED_STIRP_BRIGNTNESS
        .brightness = LED_BRIGHTNESS
#endif
    };
    ESP_ERROR_CHECK(led_strip_init(&strip));

    led_animation_t animation;
    ESP_ERROR_CHECK(led_animation_init(&animation, LED_MATRIX_WIDTH, LED_MATRIX_HEIGHT));

    printf("Init done\n");
    while (1)
    {
        led_effect_matrix(&animation, 10);
        ESP_ERROR_CHECK(led_animation_render(&animation, &strip));
        ESP_ERROR_CHECK(led_strip_flush(&strip));
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void app_main()
{
    led_strip_install();
    xTaskCreate(test, "test", 8192, NULL, 5, NULL);
}


#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <led_effect.h>
#include <led_effects/noise1.h>
#include <led_effects/plasma_waves.h>
#include <led_effects/rainbow1.h>
#include <led_effects/waterfall.h>
#include <led_effects/dna.h>

static const char *TAG = "led_matrix";

#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO 5
#define LED_CHANNEL RMT_CHANNEL_0
#define LED_MATRIX_WIDTH  16
#define LED_MATRIX_HEIGHT 16
#define LED_BRIGHTNESS 20 // 0..255
#define FPS 30

static led_strip_t strip = {
    .type = LED_TYPE,
    .length = LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT,
    .gpio = LED_GPIO,
    .channel = LED_CHANNEL,
    .buf = NULL,
#ifdef LED_STIRP_BRIGNTNESS
    .brightness = LED_BRIGHTNESS
#endif
};

// renderer from led_effect frame buffer to actual LED strip
static esp_err_t render_frame(led_effect_t *state, void *arg)
{
    for (size_t y = 0; y < state->height; y++)
        for (size_t x = 0; x < state->width; x++)
        {
            // calculate strip index of pixel
            size_t strip_idx = y * state->width + (y % 2 ? state->width - x - 1 : x);
            // find pixel offset in state frame buffer
            uint8_t *pixel = state->frame_buf + LED_EFFECT_FRAME_BUF_OFFS(state, x, y);
            // set pixel in strip
            if (state->buf_type == LED_EFFECT_RGB)
                led_strip_set_pixel(&strip, strip_idx, *((rgb_t *)pixel));
            else
                led_strip_set_pixel(&strip, strip_idx, hsv2rgb_rainbow(*((hsv_t *)pixel)));
        }

    // flush strip buffer
    return led_strip_flush(&strip);
}

// timer callback
static void display_frame(void *arg)
{
    led_effect_t *state = (led_effect_t *)arg;
    //ESP_ERROR_CHECK(led_effect_noise1_run(state));
    //ESP_ERROR_CHECK(led_effect_plasma_waves_run(state));
    //ESP_ERROR_CHECK(led_effect_rainbow1_run(state));
    //ESP_ERROR_CHECK(led_effect_waterfall_run(state));
    ESP_ERROR_CHECK(led_effect_dna_run(state));
    ESP_ERROR_CHECK(led_effect_render(state, &strip));
}

void test(void *pvParameters)
{
    // setup strip
    ESP_ERROR_CHECK(led_strip_init(&strip));
    ESP_LOGI(TAG, "LED strip initialized");

    // Setup effect
    led_effect_t effect;
    ESP_ERROR_CHECK(led_effect_init(&effect, LED_MATRIX_WIDTH, LED_MATRIX_HEIGHT, LED_EFFECT_RGB, render_frame));

    // Init effect
    //ESP_ERROR_CHECK(led_effect_noise1_init(&effect, 30, 8));
    //ESP_ERROR_CHECK(led_effect_plasma_waves_init(&effect, 235));
    //ESP_ERROR_CHECK(led_effect_rainbow1_init(&effect, RAINBOW1_HORIZONTAL, 150, 30));
    //ESP_ERROR_CHECK(led_effect_waterfall_init(&effect, WATERFALL_FIRE, 150, 90, 80));
    ESP_ERROR_CHECK(led_effect_dna_init(&effect, 30, 8, true));
    ESP_LOGI(TAG, "Effect initialized");

    // setup timer
    esp_timer_create_args_t timer_args = {
        .name = "effect",
        .arg  = &effect,
        .callback = display_frame,
        .dispatch_method = ESP_TIMER_TASK
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / FPS));
    ESP_LOGI(TAG, "Frame timer started");

    while (1)
    {
        ESP_LOGI(TAG, "Total frames rendered: %d", effect.frame_num);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    led_strip_install();
    xTaskCreate(test, "test", 8192, NULL, 5, NULL);
}


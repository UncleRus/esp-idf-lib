#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <lib8tion.h>
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

#define SWITCH_PERIOD_MS 5000

typedef enum {
    EFFECT_NONE = 0,

    EFFECT_DNA,
    EFFECT_NOISE1,
    EFFECT_WATERFALL_FIRE,
    EFFECT_WATERFALL_SIMPLE,
    EFFECT_WATERFALL_COLORS,
    EFFECT_PLASMA_WAVES,
    EFFECT_RAINBOW1,

    EFFECT_MAX
} effect_t;

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

static esp_timer_handle_t timer;

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

static effect_t current_effect = EFFECT_NONE;

// timer callback
static void display_frame(void *arg)
{
    led_effect_t *state = (led_effect_t *)arg;
    switch(current_effect)
    {
        case EFFECT_DNA:
            led_effect_dna_run(state);
            break;
        case EFFECT_NOISE1:
            led_effect_noise1_run(state);
            break;
        case EFFECT_WATERFALL_FIRE:
        case EFFECT_WATERFALL_SIMPLE:
        case EFFECT_WATERFALL_COLORS:
            led_effect_waterfall_run(state);
            break;
        case EFFECT_PLASMA_WAVES:
            led_effect_plasma_waves_run(state);
            break;
        case EFFECT_RAINBOW1:
            led_effect_rainbow1_run(state);
            break;
        default:
            return;
    }
    led_effect_render(state, &strip);
}

static void switch_effect(led_effect_t *state)
{
    // stop rendering
    esp_timer_stop(timer);

    // finish current effect
    switch(current_effect)
    {
        case EFFECT_DNA:
            led_effect_dna_done(state);
            break;
        case EFFECT_NOISE1:
            led_effect_noise1_done(state);
            break;
        case EFFECT_WATERFALL_FIRE:
        case EFFECT_WATERFALL_SIMPLE:
        case EFFECT_WATERFALL_COLORS:
            led_effect_waterfall_done(state);
            break;
        case EFFECT_PLASMA_WAVES:
            led_effect_plasma_waves_done(state);
            break;
        case EFFECT_RAINBOW1:
            led_effect_rainbow1_done(state);
            break;
        default:
            break;
    }

    // clear framebuffer
    led_effect_clear(state);

    current_effect = random8_between(EFFECT_DNA, EFFECT_MAX);

    // init new effect
    switch(current_effect)
    {
        case EFFECT_DNA:
            led_effect_dna_init(state, random8_between(10, 100), random8_between(1, 10), random8_to(2));
            break;
        case EFFECT_NOISE1:
            led_effect_noise1_init(state, random8_between(10, 100), random8_between(1, 50));
            break;
        case EFFECT_WATERFALL_FIRE:
            led_effect_waterfall_init(state, WATERFALL_FIRE, 0, random8_between(20, 120), random8_between(50, 200));
            break;
        case EFFECT_WATERFALL_SIMPLE:
            led_effect_waterfall_init(state, WATERFALL_SIMPLE, random8_between(1, 255), random8_between(20, 120), random8_between(50, 200));
            break;
        case EFFECT_WATERFALL_COLORS:
            led_effect_waterfall_init(state, WATERFALL_COLORS, random8_between(1, 255), random8_between(20, 120), random8_between(50, 200));
            break;
        case EFFECT_PLASMA_WAVES:
            led_effect_plasma_waves_init(state, random8_between(50, 255));
            break;
        case EFFECT_RAINBOW1:
            led_effect_rainbow1_init(state, random8_to(3), random8_between(10, 50), random8_between(1, 50));
            break;
        default:
            break;
    }
    esp_timer_start_periodic(timer, 1000000 / FPS);
}

void test(void *pvParameters)
{
    // setup strip
    led_strip_init(&strip);
    ESP_LOGI(TAG, "LED strip initialized");

    // Setup framebuffer
    led_effect_t effect;
    led_effect_init(&effect, LED_MATRIX_WIDTH, LED_MATRIX_HEIGHT, LED_EFFECT_RGB, render_frame);

    // setup timer
    esp_timer_create_args_t timer_args = {
        .name = "effect",
        .arg  = &effect,
        .callback = display_frame,
        .dispatch_method = ESP_TIMER_TASK
    };
    esp_timer_create(&timer_args, &timer);

    while (1)
    {
        switch_effect(&effect);
        ESP_LOGI(TAG, "Switching to effect: %d", current_effect);
        vTaskDelay(pdMS_TO_TICKS(SWITCH_PERIOD_MS));
    }
}

void app_main()
{
    rand16seed = esp_random();
    led_strip_install();
    xTaskCreate(test, "test", 8192, NULL, 5, NULL);
}


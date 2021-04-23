#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <lib8tion.h>
#include <led_effect.h>
#include <led_animation.h>

#include <led_effects/noise1.h>
#include <led_effects/plasma_waves.h>
#include <led_effects/rainbow1.h>
#include <led_effects/waterfall.h>
#include <led_effects/dna.h>
#include <led_effects/rays.h>
#include <led_effects/crazybees.h>

static const char *TAG = "led_matrix";

#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO 5
#define LED_CHANNEL RMT_CHANNEL_0
#define LED_MATRIX_WIDTH  16
#define LED_MATRIX_HEIGHT 16
#define LED_BRIGHTNESS 20 // 0..255
#define FPS 30

#define SWITCH_PERIOD_MS 5000

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)

typedef enum {
    EFFECT_NONE = 0,

    EFFECT_DNA,
    EFFECT_NOISE1,
    EFFECT_WATERFALL_FIRE,
    EFFECT_WATERFALL_SIMPLE,
    EFFECT_WATERFALL_COLORS,
    EFFECT_PLASMA_WAVES,
    EFFECT_RAINBOW1,
    EFFECT_RAYS,
    EFFECT_CRAZYBEES,

    EFFECT_MAX
} effect_t;

// renderer from led_effect frame buffer to actual LED strip
// this can be easily adapted to led_strip_spi
static esp_err_t render_frame(led_effect_t *state, void *arg)
{
    if (!arg)
        return ESP_ERR_INVALID_ARG;

    led_strip_t *led_strip = (led_strip_t *)arg;

    for (size_t y = 0; y < state->height; y++)
        for (size_t x = 0; x < state->width; x++)
        {
            // calculate strip index of pixel
            size_t strip_idx = y * state->width + (y % 2 ? state->width - x - 1 : x);
            // find pixel offset in state frame buffer
            rgb_t color = state->frame_buf[LED_EFFECT_FRAMEBUF_OFFS(state, x, y)];
            // limit brightness and consuming current
            color = rgb_scale_video(color, LED_BRIGHTNESS);
            CHECK(led_strip_set_pixel(led_strip, strip_idx, color));
        }

    // flush strip buffer
    return led_strip_flush(led_strip);
}

static led_strip_t strip = {
    .type = LED_TYPE,
    .length = LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT,
    .gpio = LED_GPIO,
    .channel = LED_CHANNEL,
    .buf = NULL,
#ifdef LED_STIRP_BRIGNTNESS
    .brightness = 0xff
#endif
};

static effect_t current_effect = EFFECT_NONE;

static void switch_effect(led_effect_animation_t *animation)
{
    // stop rendering
    if (current_effect != EFFECT_NONE)
        led_effect_animation_stop(animation);

    // finish current effect
    switch(current_effect)
    {
        case EFFECT_DNA:
            led_effect_dna_done(animation->state);
            break;
        case EFFECT_NOISE1:
            led_effect_noise1_done(animation->state);
            break;
        case EFFECT_WATERFALL_FIRE:
        case EFFECT_WATERFALL_SIMPLE:
        case EFFECT_WATERFALL_COLORS:
            led_effect_waterfall_done(animation->state);
            break;
        case EFFECT_PLASMA_WAVES:
            led_effect_plasma_waves_done(animation->state);
            break;
        case EFFECT_RAINBOW1:
            led_effect_rainbow1_done(animation->state);
            break;
        case EFFECT_RAYS:
            led_effect_rays_done(animation->state);
            break;
        case EFFECT_CRAZYBEES:
            led_effect_crazybees_done(animation->state);
            break;
        default:
            break;
    }

    // clear framebuffer
    led_effect_clear(animation->state);

    // pick new effect
    current_effect = random8_between(EFFECT_NONE + 1, EFFECT_MAX);

    // init new effect
    led_effect_run_cb_t effect_func = NULL;
    switch(current_effect)
    {
        case EFFECT_DNA:
            led_effect_dna_init(animation->state, random8_between(10, 100), random8_between(1, 10), random8_to(2));
            effect_func = led_effect_dna_run;
            break;
        case EFFECT_NOISE1:
            led_effect_noise1_init(animation->state, random8_between(10, 100), random8_between(1, 50));
            effect_func = led_effect_noise1_run;
            break;
        case EFFECT_WATERFALL_FIRE:
            led_effect_waterfall_init(animation->state, WATERFALL_FIRE, 0, random8_between(20, 120), random8_between(50, 200));
            effect_func = led_effect_waterfall_run;
            break;
        case EFFECT_WATERFALL_SIMPLE:
            led_effect_waterfall_init(animation->state, WATERFALL_SIMPLE, random8_between(1, 255), random8_between(20, 120), random8_between(50, 200));
            effect_func = led_effect_waterfall_run;
            break;
        case EFFECT_WATERFALL_COLORS:
            led_effect_waterfall_init(animation->state, WATERFALL_COLORS, random8_between(1, 255), random8_between(20, 120), random8_between(50, 200));
            effect_func = led_effect_waterfall_run;
            break;
        case EFFECT_PLASMA_WAVES:
            led_effect_plasma_waves_init(animation->state, random8_between(50, 255));
            effect_func = led_effect_plasma_waves_run;
            break;
        case EFFECT_RAINBOW1:
            led_effect_rainbow1_init(animation->state, random8_to(3), random8_between(10, 50), random8_between(1, 50));
            effect_func = led_effect_rainbow1_run;
            break;
        case EFFECT_RAYS:
            led_effect_rays_init(animation->state, random8_between(0, 50), random8_between(3, 5), random8_between(5, 10));
            effect_func = led_effect_rays_run;
            break;
        case EFFECT_CRAZYBEES:
            led_effect_crazybees_init(animation->state, random8_between(1, 5));
            effect_func = led_effect_crazybees_run;
            break;
        default:
            break;
    }

    // start rendering
    led_effect_animation_play(animation, FPS, effect_func, &strip);
}

void test(void *pvParameters)
{
    // setup strip
    led_strip_init(&strip);
    ESP_LOGI(TAG, "LED strip initialized");

    // Setup framebuffer
    led_effect_t effect;
    led_effect_init(&effect, LED_MATRIX_WIDTH, LED_MATRIX_HEIGHT, render_frame);

    // setup animation
    led_effect_animation_t animation;
    led_effect_animation_init(&animation, &effect);

    while (1)
    {
        switch_effect(&animation);
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


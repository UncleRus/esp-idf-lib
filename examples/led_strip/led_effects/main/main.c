#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <led_strip.h>
#include <lib8tion.h>
#include <framebuffer.h>
#include <fbanimation.h>

#include <effects/noise.h>
#include <effects/plasma_waves.h>
#include <effects/rainbow.h>
#include <effects/waterfall.h>
#include <effects/dna.h>
#include <effects/rays.h>
#include <effects/crazybees.h>
#include <effects/sparkles.h>
#include <effects/matrix.h>
#include <effects/rain.h>
#include <effects/fire.h>

static const char *TAG = "led_effect_example";

#define LED_GPIO CONFIG_EXAMPLE_LED_GPIO
#define LED_TYPE LED_STRIP_WS2812

#define LED_MATRIX_WIDTH  CONFIG_EXAMPLE_LED_MATRIX_WIDTH
#define LED_MATRIX_HEIGHT CONFIG_EXAMPLE_LED_MATRIX_HEIGHT
#define LED_BRIGHTNESS CONFIG_EXAMPLE_LED_BRIGHTNESS // 0..255

#define FPS CONFIG_EXAMPLE_FPS

#define SWITCH_PERIOD_MS CONFIG_EXAMPLE_SWITCH_PERIOD_MS

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)

typedef enum {
    EFFECT_NONE = 0,

    EFFECT_DNA,
    EFFECT_NOISE,
    EFFECT_WATERFALL_FIRE,
    EFFECT_WATERFALL,
    EFFECT_PLASMA_WAVES,
    EFFECT_RAINBOW,
    EFFECT_RAYS,
    EFFECT_CRAZYBEES,
    EFFECT_SPARKLES,
    EFFECT_MATRIX,
    EFFECT_RAIN,
    EFFECT_FIRE,

    EFFECT_MAX
} effect_t;

// renderer from framebuffer to actual LED strip
// this can be easily adapted to led_strip_spi or any display
static esp_err_t render_frame(framebuffer_t *fb, void *arg)
{
    if (!arg)
        return ESP_ERR_INVALID_ARG;

    led_strip_t *led_strip = (led_strip_t *)arg;

    for (size_t y = 0; y < fb->height; y++)
        for (size_t x = 0; x < fb->width; x++)
        {
            // calculate strip index of pixel
            size_t strip_idx = y * fb->width + (y % 2 ? fb->width - x - 1 : x);
            // find pixel offset in state frame buffer
            rgb_t color = fb->data[FB_OFFSET(fb, x, y)];
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
    .buf = NULL,
#ifdef LED_STRIP_BRIGHTNESS
    .brightness = 255,
#endif
};

static effect_t current_effect = EFFECT_NONE;
static fb_draw_cb_t effect_done = NULL;

static void switch_effect(fb_animation_t *animation)
{
    // stop rendering
    if (current_effect != EFFECT_NONE)
        fb_animation_stop(animation);

    // finish current effect
    if (effect_done)
        effect_done(animation->fb);

    // clear framebuffer
    fb_clear(animation->fb);

    // pick new effect
    if (++current_effect == EFFECT_MAX)
        current_effect = EFFECT_NONE + 1;

    // init new effect
    fb_draw_cb_t effect_func = NULL;
    switch(current_effect)
    {
        case EFFECT_DNA:
            led_effect_dna_init(animation->fb, random8_between(10, 100), random8_between(1, 10), random8_to(2));
            effect_func = led_effect_dna_run;
            effect_done = led_effect_dna_done;
            break;
        case EFFECT_NOISE:
            led_effect_noise_init(animation->fb, random8_between(10, 100), random8_between(1, 50));
            effect_func = led_effect_noise_run;
            effect_done = led_effect_noise_done;
            break;
        case EFFECT_WATERFALL_FIRE:
            led_effect_waterfall_init(animation->fb, random8_between(2, 4), 0, random8_between(20, 120), random8_between(50, 200));
            effect_func = led_effect_waterfall_run;
            effect_done = led_effect_waterfall_done;
            break;
        case EFFECT_WATERFALL:
            led_effect_waterfall_init(animation->fb, random8_to(2), random8_between(1, 255), random8_between(20, 120), random8_between(50, 200));
            effect_func = led_effect_waterfall_run;
            effect_done = led_effect_waterfall_done;
            break;
        case EFFECT_PLASMA_WAVES:
            led_effect_plasma_waves_init(animation->fb, random8_between(50, 255));
            effect_func = led_effect_plasma_waves_run;
            effect_done = led_effect_plasma_waves_done;
            break;
        case EFFECT_RAINBOW:
            led_effect_rainbow_init(animation->fb, random8_to(3), random8_between(10, 50), random8_between(1, 20));
            effect_func = led_effect_rainbow_run;
            effect_done = led_effect_rainbow_done;
            break;
        case EFFECT_RAYS:
            led_effect_rays_init(animation->fb, random8_between(0, 50), random8_between(3, 5), random8_between(5, 10));
            effect_func = led_effect_rays_run;
            effect_done = led_effect_rays_done;
            break;
        case EFFECT_CRAZYBEES:
            led_effect_crazybees_init(animation->fb, random8_between(2, 5));
            effect_func = led_effect_crazybees_run;
            effect_done = led_effect_crazybees_done;
            break;
        case EFFECT_SPARKLES:
            led_effect_sparkles_init(animation->fb, random8_between(1, 20), random8_between(10, 150));
            effect_func = led_effect_sparkles_run;
            effect_done = led_effect_sparkles_done;
            break;
        case EFFECT_MATRIX:
            led_effect_matrix_init(animation->fb, random8_between(10, 250));
            effect_func = led_effect_matrix_run;
            effect_done = led_effect_matrix_done;
            break;
        case EFFECT_RAIN:
            led_effect_rain_init(animation->fb, random8_to(2), random8(), random8_to(100), random8_between(100, 200));
            effect_func = led_effect_rain_run;
            effect_done = led_effect_rain_done;
            break;
        case EFFECT_FIRE:
            led_effect_fire_init(animation->fb, random8_to(3));
            effect_func = led_effect_fire_run;
            effect_done = led_effect_fire_done;
            break;
        default:
            break;
    }

    // start rendering
    fb_animation_play(animation, FPS, effect_func, &strip);
}

void test(void *pvParameters)
{
    // setup strip
    led_strip_init(&strip);
    ESP_LOGI(TAG, "LED strip initialized");

    // Setup framebuffer
    framebuffer_t fb;
    fb_init(&fb, LED_MATRIX_WIDTH, LED_MATRIX_HEIGHT, render_frame);

    // setup animation
    fb_animation_t animation;
    fb_animation_init(&animation, &fb);

    while (1)
    {
        switch_effect(&animation);
        ESP_LOGI(TAG, "Switching to effect: %d", current_effect);
        vTaskDelay(pdMS_TO_TICKS(SWITCH_PERIOD_MS));
    }
}

void app_main()
{
    led_strip_install();
    xTaskCreate(test, "test", 8192, NULL, 5, NULL);
}


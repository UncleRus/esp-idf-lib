#include <esp_err.h>
#include <esp_log.h>
#include "led_animation.h"

static const char *TAG = "led_animation";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static void display_frame(void *ctx)
{
    led_effect_animation_t *animation = (led_effect_animation_t *)ctx;

    // run effect
    esp_err_t res = animation->effect_func ? animation->effect_func(animation->state) : ESP_FAIL;
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error running effect %d (%s)", res, esp_err_to_name(res));
        return;
    }
    // render frame
    led_effect_render_frame(animation->state, animation->render_ctx);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error rendering frame %d (%s)", res, esp_err_to_name(res));
        return;
    }
}

esp_err_t led_effect_animation_init(led_effect_animation_t *animation, led_effect_t *state)
{
    CHECK_ARG(animation && state);

    animation->state = state;
    animation->timer = NULL;
    esp_timer_create_args_t timer_args = {
        .arg = animation,
        .callback = display_frame,
        .dispatch_method = ESP_TIMER_TASK
    };
    return esp_timer_create(&timer_args, &animation->timer);
}

esp_err_t led_effect_animation_play(led_effect_animation_t *animation, uint8_t fps,
        led_effect_run_cb_t effect_func, void *render_ctx)
{
    CHECK_ARG(animation && fps && effect_func);

    animation->render_ctx = render_ctx;
    animation->effect_func = effect_func;
    return esp_timer_start_periodic(animation->timer, 1000000 / fps);
}

esp_err_t led_effect_animation_stop(led_effect_animation_t *animation)
{
    CHECK_ARG(animation);

    return esp_timer_stop(animation->timer);
}

esp_err_t led_effect_animation_free(led_effect_animation_t *animation)
{
    CHECK_ARG(animation);

    esp_timer_stop(animation->timer);
    return esp_timer_delete(animation->timer);
}


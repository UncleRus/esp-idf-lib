/**
 * @file encoder.c
 *
 * ESP-IDF HW timer-based driver for rotary encoders
 *
 * Copyright (C) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "encoder.h"
#include <esp_log.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_timer.h>

ESP_EVENT_DEFINE_BASE(RE_EVENT);

#define MUTEX_TIMEOUT 10

#ifdef CONFIG_RE_BTN_PRESSED_LEVEL_0
    #define CONFIG_RE_BTN_PRESSED_LEVEL 0
#else
    #define CONFIG_RE_BTN_PRESSED_LEVEL 1
#endif

static const char *TAG = "ENCODER";
static rotary_encoder_t *encs[CONFIG_RE_MAX] = { 0 };
static const int8_t valid_states[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
static SemaphoreHandle_t mutex;
static esp_event_loop_handle_t loop;

#define GPIO_BIT(x) ((x) < 32 ? BIT(x) : ((uint64_t)(((uint64_t)1)<<(x))))
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

inline static void read_encoder(rotary_encoder_t *re)
{
    if (re->pin_btn < GPIO_NUM_MAX)
    do
    {
        if (re->btn_state == RE_BTN_PRESSED && re->btn_pressed_time_us < CONFIG_RE_BTN_DEAD_TIME_US)
        {
            // Dead time
            re->btn_pressed_time_us += CONFIG_RE_INTERVAL_US;
            break;
        }

        // read button state
        if (gpio_get_level(re->pin_btn) == CONFIG_RE_BTN_PRESSED_LEVEL)
        {
            if (re->btn_state == RE_BTN_RELEASED)
            {
                // first press
                re->btn_state = RE_BTN_PRESSED;
                re->btn_pressed_time_us = 0;
                esp_event_post_to(loop, RE_EVENT, RE_EVENT_BTN_PRESSED, &re, sizeof(re), 0);
                break;
            }

            re->btn_pressed_time_us += CONFIG_RE_INTERVAL_US;

            if (re->btn_state == RE_BTN_PRESSED && re->btn_pressed_time_us >= CONFIG_RE_BTN_LONG_PRESS_TIME_US)
            {
                // Long press
                re->btn_state = RE_BTN_LONG_PRESSED;
                esp_event_post_to(loop, RE_EVENT, RE_EVENT_BTN_LONG_PRESSED, &re, sizeof(re), 0);
            }
        }
        else if (re->btn_state != RE_BTN_RELEASED)
        {
            bool clicked = re->btn_state == RE_BTN_PRESSED;
            // released
            re->btn_state = RE_BTN_RELEASED;
            esp_event_post_to(loop, RE_EVENT, RE_EVENT_BTN_RELEASED, &re, sizeof(re), 0);
            if (clicked)
                esp_event_post_to(loop, RE_EVENT, RE_EVENT_BTN_CLICKED, &re, sizeof(re), 0);
        }
    } while(0);

    re->code <<= 2;
    re->code |= gpio_get_level(re->pin_a);
    re->code |= gpio_get_level(re->pin_b) << 1;
    re->code &= 0xf;

    if (!valid_states[re->code])
        return;

    re->store = (re->store << 4) | re->code;

    switch (re->store)
    {
        case 0xe817:
            re->diff = 1;
            break;
        case 0xd42b:
            re->diff = -1;
            break;
        default:
            re->diff = 0;
    }
    if (re->diff)
        esp_event_post_to(loop, RE_EVENT, RE_EVENT_CHANGED, &re, sizeof(re), 0);
}

static void timer_handler(void *arg)
{
    if (!xSemaphoreTake(mutex, 0))
        return;

    for (size_t i = 0; i < CONFIG_RE_MAX; i++)
        if (encs[i])
            read_encoder(encs[i]);

    xSemaphoreGive(mutex);
}

static const esp_timer_create_args_t timer_args = {
        .name = "__encoder__",
        .arg = NULL,
        .callback = timer_handler,
        .dispatch_method = ESP_TIMER_TASK
};

static esp_timer_handle_t timer;

esp_err_t rotary_encoder_init(esp_event_loop_handle_t event_loop)
{
    CHECK_ARG(event_loop);
    loop = event_loop;

    mutex = xSemaphoreCreateMutex();
    if (!mutex)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    CHECK(esp_timer_create(&timer_args, &timer));
    CHECK(esp_timer_start_periodic(timer, CONFIG_RE_INTERVAL_US));

    ESP_LOGI(TAG, "Initialization complete, timer interval: %dms", CONFIG_RE_INTERVAL_US / 1000);
    return ESP_OK;
}

esp_err_t rotary_encoder_add(rotary_encoder_t *re)
{
    CHECK_ARG(re);
    if (!xSemaphoreTake(mutex, MUTEX_TIMEOUT))
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_INVALID_STATE;
    }

    bool ok = false;
    for (size_t i = 0; i < CONFIG_RE_MAX; i++)
        if (!encs[i])
        {
            re->index = i;
            encs[i] = re;
            ok = true;
            break;
        }
    if (!ok)
    {
        ESP_LOGE(TAG, "Too many encoders");
        xSemaphoreGive(mutex);
        return ESP_ERR_NO_MEM;
    }

    // setup GPIO
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(gpio_config_t));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_BIT(re->pin_a) | GPIO_BIT(re->pin_b);
    if (re->pin_btn < GPIO_NUM_MAX)
        io_conf.pin_bit_mask |= GPIO_BIT(re->pin_btn);
    CHECK(gpio_config(&io_conf));

    re->btn_state = RE_BTN_RELEASED;
    re->btn_pressed_time_us = 0;

    xSemaphoreGive(mutex);

    ESP_LOGI(TAG, "Added rotary encoder %d, A: %d, B: %d, BTN: %d", re->index, re->pin_a, re->pin_b, re->pin_btn);
    return ESP_OK;
}

esp_err_t rotary_encoder_remove(rotary_encoder_t *re)
{
    CHECK_ARG(re);
    if (!xSemaphoreTake(mutex, MUTEX_TIMEOUT))
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < CONFIG_RE_MAX; i++)
        if (encs[i] == re)
        {
            encs[i] = NULL;
            ESP_LOGI(TAG, "Removed rotary encoder %d", i);
            xSemaphoreGive(mutex);
            return ESP_OK;
        }

    ESP_LOGE(TAG, "Unknown encoder");
    xSemaphoreGive(mutex);
    return ESP_ERR_NOT_FOUND;
}

/**
 * @file encoder.c
 *
 * ESP-IDF timer-based driver for rotary encoders
 *
 * Copyright (C) 2019 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#include "encoder.h"
#include <esp_log.h>
#include <soc/timer_group_struct.h>
#include <driver/periph_ctrl.h>
#include <driver/timer.h>
#include <string.h>
#include <freertos/semphr.h>

#define MUTEX_TIMEOUT 10

#define TIMER_DIVIDER (APB_CLK_FREQ / 1000000)

#ifdef CONFIG_RE_BTN_PRESSED_LEVEL_0
    #define CONFIG_RE_BTN_PRESSED_LEVEL 0
#else
    #define CONFIG_RE_BTN_PRESSED_LEVEL 1
#endif

#ifdef CONFIG_RE_TGROUP_0
    #define CONFIG_RE_TGROUP 0
#else
    #define CONFIG_RE_TGROUP 1
#endif

#ifdef CONFIG_RE_TIMER_0
    #define CONFIG_RE_TIMER 0
#else
    #define CONFIG_RE_TIMER 1
#endif

#if (CONFIG_RE_TGROUP == 0)
    #define TIMERG TIMERG0
#else
    #define TIMERG TIMERG1
#endif

static const char *TAG = "ENCODER";
static rotary_encoder_t *encs[CONFIG_RE_MAX] = { 0 };
static const int8_t valid_states[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
static SemaphoreHandle_t mutex;
static QueueHandle_t _queue;

#define GPIO_BIT(x) ((x) < 32 ? BIT(x) : ((uint64_t)(((uint64_t)1)<<(x))))
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

inline static void read_encoder(rotary_encoder_t *re)
{
    rotary_encoder_event_t ev = {
        .sender = re
    };

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
                ev.type = RE_ET_BTN_PRESSED;
                xQueueSendFromISR(_queue, &ev, NULL);
                break;
            }

            re->btn_pressed_time_us += CONFIG_RE_INTERVAL_US;

            if (re->btn_state == RE_BTN_PRESSED && re->btn_pressed_time_us >= CONFIG_RE_BTN_LONG_PRESS_TIME_US)
            {
                // Long press
                re->btn_state = RE_BTN_LONG_PRESSED;
                ev.type = RE_ET_BTN_LONG_PRESSED;
                xQueueSendFromISR(_queue, &ev, NULL);
            }
        }
        else if (re->btn_state != RE_BTN_RELEASED)
        {
            // released
            re->btn_state = RE_BTN_RELEASED;
            ev.type = RE_ET_BTN_RELEASED;
            xQueueSendFromISR(_queue, &ev, NULL);
        }
    } while(0);

    re->code <<= 2;
    re->code |= gpio_get_level(re->pin_a);
    re->code |= gpio_get_level(re->pin_b) << 1;
    re->code &= 0xf;

    if (!valid_states[re->code])
        return;

    int8_t inc = 0;

    re->store = (re->store << 4) | re->code;
//    if (re->store == 0x2b || re->store == 0xd4) inc = -re->step;
//    if (re->store == 0x17 || re->store == 0xe8) inc = re->step;
    if (re->store == 0x2b || re->store == 0xd4) inc = -1;
    if (re->store == 0x17 || re->store == 0xe8) inc = 1;

//    if ((re->min != re->max) && ((re->value <= re->min && inc < 0) || (re->value >= re->max && inc > 0)))
//        return;

//    re->value += inc;
    if (inc)
    {
        ev.type = RE_ET_CHANGED;
        ev.diff = inc;
        xQueueSendFromISR(_queue, &ev, NULL);
    }
}

static void timer_isr(void *arg)
{
    timer_set_alarm(CONFIG_RE_TGROUP, CONFIG_RE_TIMER, TIMER_ALARM_EN);
#if (CONFIG_RE_TIMER == 0)
    TIMERG.int_clr_timers.t0 = 1;
#else
    TIMERG.int_clr_timers.t1 = 1;
#endif

    if (!xSemaphoreTakeFromISR(mutex, NULL))
        return;

    for (size_t i = 0; i < CONFIG_RE_MAX; i++)
        if (encs[i])
            read_encoder(encs[i]);

    xSemaphoreGiveFromISR(mutex, NULL);
}

esp_err_t rotary_encoder_init(QueueHandle_t queue)
{
    CHECK_ARG(queue);
    _queue = queue;

    mutex = xSemaphoreCreateMutex();
    if (!mutex)
    {
        ESP_LOGE(TAG, "Could not create mutex");
        return ESP_ERR_NO_MEM;
    }

    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;
    CHECK(timer_init(CONFIG_RE_TGROUP, CONFIG_RE_TIMER, &config));

    CHECK(timer_set_counter_value(CONFIG_RE_TGROUP, CONFIG_RE_TIMER, 0));
    CHECK(timer_set_alarm_value(CONFIG_RE_TGROUP, CONFIG_RE_TIMER, CONFIG_RE_INTERVAL_US));
    CHECK(timer_enable_intr(CONFIG_RE_TGROUP, CONFIG_RE_TIMER));
    CHECK(timer_isr_register(CONFIG_RE_TGROUP, CONFIG_RE_TIMER, timer_isr, NULL, 0, NULL));
    CHECK(timer_start(CONFIG_RE_TGROUP, CONFIG_RE_TIMER));

    ESP_LOGI(TAG, "Initialization complete, timer %d:%d, interval: %dms", CONFIG_RE_TGROUP, CONFIG_RE_TIMER,
            CONFIG_RE_INTERVAL_US / 1000);
    return ESP_OK;
}

esp_err_t rotary_encoder_add(rotary_encoder_t *re)
{
    CHECK_ARG(re);
    if (!xSemaphoreTake(mutex, MUTEX_TIMEOUT))
    {
        ESP_LOGE(TAG, "Could not take mutex");
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
        ESP_LOGE(TAG, "Could not take mutex");
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

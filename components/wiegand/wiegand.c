/**
 * @file wiegand.c
 *
 * ESP-IDF Wiegand protocol receiver
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>
#include <esp_idf_lib_helpers.h>
#include "wiegand.h"

static const char *TAG = "wiegand";

#define TIMER_INTERVAL_US 50000 // 50ms

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static void isr_disable(wiegand_reader_t *reader)
{
    gpio_set_intr_type(reader->gpio_d0, GPIO_INTR_DISABLE);
    gpio_set_intr_type(reader->gpio_d1, GPIO_INTR_DISABLE);
}

static void isr_enable(wiegand_reader_t *reader)
{
    gpio_set_intr_type(reader->gpio_d0, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(reader->gpio_d1, GPIO_INTR_NEGEDGE);
}

#if HELPER_TARGET_IS_ESP32
static void IRAM_ATTR isr_handler(void *arg)
#else
static void isr_handler(void *arg)
#endif
{
    wiegand_reader_t *reader = (wiegand_reader_t *)arg;

    int d0 = gpio_get_level(reader->gpio_d0);
    int d1 = gpio_get_level(reader->gpio_d1);

    // ignore equal
    if (d0 == d1)
        return;
    // overflow
    if (reader->bits >= reader->size * 8)
        return;

    esp_timer_stop(reader->timer);

    uint8_t value = d0 ? 0x80 : 0;

    reader->buf[reader->bits / 8] |= value >> (reader->bits % 8);
    reader->bits++;

    esp_timer_start_once(reader->timer, TIMER_INTERVAL_US);
}

static void timer_handler(void *arg)
{
    wiegand_reader_t *reader = (wiegand_reader_t *)arg;

    ESP_LOGD(TAG, "Got %d bits of data", reader->bits);

    isr_disable(reader);

    if (reader->callback)
        reader->callback(reader);

    reader->bits = 0;
    memset(reader->buf, 0, reader->size);

    isr_enable(reader);
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t wiegand_reader_init(wiegand_reader_t *reader, gpio_num_t gpio_d0, gpio_num_t gpio_d1,
        bool internal_pullups, size_t buf_size, wiegand_callback_t callback)
{
    CHECK_ARG(reader && buf_size && callback);

    esp_err_t res = gpio_install_isr_service(0);
    if (res != ESP_OK && res != ESP_ERR_INVALID_STATE)
        return res;

    memset(reader, 0, sizeof(wiegand_reader_t));
    reader->gpio_d0 = gpio_d0;
    reader->gpio_d1 = gpio_d1;
    reader->size = buf_size;
    reader->buf = calloc(buf_size, 1);
    reader->callback = callback;

    esp_timer_create_args_t timer_args = {
        .name = TAG,
        .arg = reader,
        .callback = timer_handler,
        .dispatch_method = ESP_TIMER_TASK
    };
    CHECK(esp_timer_create(&timer_args, &reader->timer));

    CHECK(gpio_set_direction(gpio_d0, GPIO_MODE_INPUT));
    CHECK(gpio_set_direction(gpio_d1, GPIO_MODE_INPUT));
    CHECK(gpio_set_pull_mode(gpio_d0, internal_pullups ? GPIO_PULLUP_ONLY : GPIO_FLOATING));
    CHECK(gpio_set_pull_mode(gpio_d1, internal_pullups ? GPIO_PULLUP_ONLY : GPIO_FLOATING));
    isr_disable(reader);
    CHECK(gpio_isr_handler_add(gpio_d0, isr_handler, reader));
    CHECK(gpio_isr_handler_add(gpio_d1, isr_handler, reader));
    isr_enable(reader);

    ESP_LOGI(TAG, "Reader initialized on D0=%d, D1=%d", gpio_d0, gpio_d1);

    return ESP_OK;
}

esp_err_t wiegand_reader_done(wiegand_reader_t *reader)
{
    CHECK_ARG(reader && reader->buf);

    isr_disable(reader);
    CHECK(gpio_isr_handler_remove(reader->gpio_d0));
    CHECK(gpio_isr_handler_remove(reader->gpio_d1));
    esp_timer_stop(reader->timer);
    CHECK(esp_timer_delete(reader->timer));
    free(reader->buf);

    ESP_LOGI(TAG, "Reader removed");

    return ESP_OK;
}

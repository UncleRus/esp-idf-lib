/**
 * @file wiegand.c
 *
 *
 * Copyright (C) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <string.h>
#include "wiegand.h"

static const char *TAG = "WIEGAND";

#define TIMER_INTERVAL_US 50000 // 50ms

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static void isr_disable(wiegand_reader_t *reader)
{
    gpio_intr_disable(reader->gpio_d0);
    gpio_intr_disable(reader->gpio_d1);
}

static void isr_enable(wiegand_reader_t *reader)
{
    gpio_intr_enable(reader->gpio_d0);
    gpio_intr_enable(reader->gpio_d1);
}

static void IRAM_ATTR isr_handler(void *arg)
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

    ESP_LOGD(TAG, "(%p) Got %d bits of data", reader, reader->bits);

    if (reader->callback)
        reader->callback(reader);

    isr_disable(reader);
    reader->bits = 0;
    memset(reader->buf, 0, reader->size);
    isr_enable(reader);
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t wiegand_reader_init(wiegand_reader_t *reader, gpio_num_t gpio_d0, gpio_num_t gpio_d1,
        bool internal_pullups, size_t buf_size, wiegand_callback_t callback)
{
    CHECK_ARG(reader && buf_size && callback);

    memset(reader, 0, sizeof(wiegand_reader_t));
    reader->gpio_d0 = gpio_d0;
    reader->gpio_d1 = gpio_d1;
    reader->size = buf_size;
    reader->buf = calloc(buf_size, 1);

    char t_name[17];
    snprintf(t_name, sizeof(t_name), "wiegand_%p", reader);
    esp_timer_create_args_t timer_args = {
        .name = t_name,
        .arg = reader,
        .callback = timer_handler,
        .dispatch_method = ESP_TIMER_TASK
    };
    CHECK(esp_timer_create(&timer_args, &reader->timer));

    CHECK(gpio_set_direction(gpio_d0, GPIO_MODE_INPUT));
    CHECK(gpio_set_direction(gpio_d1, GPIO_MODE_INPUT));
    CHECK(gpio_set_pull_mode(gpio_d0, internal_pullups ? GPIO_PULLUP_ONLY : GPIO_FLOATING));
    CHECK(gpio_set_pull_mode(gpio_d1, internal_pullups ? GPIO_PULLUP_ONLY : GPIO_FLOATING));
    CHECK(gpio_set_intr_type(gpio_d0, GPIO_INTR_LOW_LEVEL));
    CHECK(gpio_set_intr_type(gpio_d1, GPIO_INTR_LOW_LEVEL));
    CHECK(gpio_isr_handler_add(gpio_d0, isr_handler, reader));
    CHECK(gpio_isr_handler_add(gpio_d1, isr_handler, reader));
    isr_enable(reader);

    ESP_LOGI(TAG, "(%p) Reader initialized on D0=%d, D1=%d", reader, gpio_d0, gpio_d1);

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

    ESP_LOGI(TAG, "(%p) Reader removed", reader);

    return ESP_OK;
}

esp_err_t wiegand_reader_decode(wiegand_reader_t *reader, wiegand_fmt_t fmt, void *res)
{
    CHECK_ARG(res && reader && reader->buf);

    switch (fmt)
    {
        case WIEGAND_26_HID:
            if (reader->bits != 26) return ESP_FAIL;
            *(uint32_t *)reader->buf <<= 1;
            ((wiegand_fmt_26_hid_t *)res)->facility = reader->buf[0];
            ((wiegand_fmt_26_hid_t *)res)->number = ((uint16_t)(reader->buf[1]) << 8) | reader->buf[2];
            ((wiegand_fmt_26_hid_t *)res)->raw = (((uint32_t)reader->buf[0]) << 16) | ((wiegand_fmt_26_hid_t *)res)->number;
            break;
        case WIEGAND_31_HID:
            if (reader->bits != 31) return ESP_FAIL;
            ((wiegand_fmt_31_hid_t *)res)->facility = (reader->buf[0] >> 3) & 0x0f;
            *(uint32_t *)reader->buf <<= 5;
            ((wiegand_fmt_31_hid_t *)res)->cardholder = ((uint32_t)(reader->buf[0]) << 16) | ((uint32_t)(reader->buf[1]) << 8) | reader->buf[2];
    }

    return ESP_OK;
}


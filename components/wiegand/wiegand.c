/**
 * @file wiegand.c
 *
 * ESP-IDF component to communicate with Wiegand reader
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>
#include "wiegand.h"

static const char *TAG = "wiegand";

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

    memset(reader, 0, sizeof(wiegand_reader_t));
    reader->gpio_d0 = gpio_d0;
    reader->gpio_d1 = gpio_d1;
    reader->size = buf_size;
    reader->buf = calloc(buf_size, 1);

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
    CHECK(gpio_set_intr_type(gpio_d0, GPIO_INTR_LOW_LEVEL));
    CHECK(gpio_set_intr_type(gpio_d1, GPIO_INTR_LOW_LEVEL));
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

////////////////////////////////////////////////////////////////////////////////
/// Decode

static inline uint8_t get_bit(wiegand_reader_t *reader, size_t pos)
{
    return (reader->buf[pos / 8] >> (7 - (pos % 8))) & 1;
}

static uint64_t get_bit_field(wiegand_reader_t *reader, size_t start_bit, size_t bits)
{
    uint64_t res = 0;
    for (size_t i = 0; i < bits; i++)
        res = (res << 1) | get_bit(reader, start_bit + i);
    return res;
}

typedef struct
{
    uint8_t bits;
    size_t issue_level_start, issue_level_len;
    size_t facility_start, facility_len;
    size_t number_start, number_len;
    size_t cardholder_start, cardholder_len;
} format_desc_t;

static const format_desc_t formats[] = {
    [WIEGAND_H10301] = {
        .bits = 26,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 1,  .facility_len    = 8,
        .number_start      = 9,  .number_len      = 16,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_2804] = {
        .bits = 28,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 4,  .facility_len    = 8,
        .number_start      = 12, .number_len      = 15,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_ATS30] = {
        .bits = 30,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 1,  .facility_len    = 12,
        .number_start      = 13, .number_len      = 16,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_ADT31] = {
        .bits = 31,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 1,  .facility_len    = 4,
        .number_start      = 0,  .number_len      = 0,
        .cardholder_start  = 5,  .cardholder_len  = 23,
    },
    [WIEGAND_KASTLE] = {
        .bits = 32,
        .issue_level_start = 2,  .issue_level_len = 5,
        .facility_start    = 7,  .facility_len    = 8,
        .number_start      = 0,  .number_len      = 0,
        .cardholder_start  = 15, .cardholder_len  = 16,
    },
    [WIEGAND_D10202] = {
        .bits = 33,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 1,  .facility_len    = 7,
        .number_start      = 8,  .number_len      = 24,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_H10306] = {
        .bits = 34,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 1,  .facility_len    = 16,
        .number_start      = 17, .number_len      = 16,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_C1000] = {
        .bits = 35,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 2,  .facility_len    = 12,
        .number_start      = 14, .number_len      = 20,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_KS36] = {
        .bits = 36,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 11, .facility_len    = 8,
        .number_start      = 19, .number_len      = 16,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_S12906] = {
        .bits = 36,
        .issue_level_start = 9,  .issue_level_len = 2,
        .facility_start    = 1,  .facility_len    = 8,
        .number_start      = 11, .number_len      = 24,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_SIEMENS] = {
        .bits = 36,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 1,  .facility_len    = 18,
        .number_start      = 19, .number_len      = 16,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_H10302] = {
        .bits = 37,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 0,  .facility_len    = 0,
        .number_start      = 1,  .number_len      = 35,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
    [WIEGAND_H10304] = {
        .bits = 37,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 1,  .facility_len    = 16,
        .number_start      = 0,  .number_len      = 0,
        .cardholder_start  = 17, .cardholder_len  = 19,
    },
    [WIEGAND_P10001] = {
        .bits = 40,
        .issue_level_start = 0,  .issue_level_len = 0,
        .facility_start    = 4,  .facility_len    = 12,
        .number_start      = 16, .number_len      = 16,
        .cardholder_start  = 0,  .cardholder_len  = 0,
    },
};

esp_err_t wiegand_reader_decode(wiegand_reader_t *reader, wiegand_format_t fmt, wiegand_card_t *card)
{
    CHECK_ARG(card && reader && reader->buf && fmt < WIEGAND_FMT_MAX);

    if (formats[fmt].bits != reader->bits)
    {
        ESP_LOGE(TAG, "Invalid bits count: expected %d, got %d", formats[fmt].bits, reader->bits);
        return ESP_FAIL;
    }

    memset(card, 0, sizeof(wiegand_card_t));

    if (formats[fmt].issue_level_len)
        card->issue_level = get_bit_field(reader, formats[fmt].issue_level_start, formats[fmt].issue_level_len);
    if (formats[fmt].facility_len)
        card->facility = get_bit_field(reader, formats[fmt].facility_start, formats[fmt].facility_len);
    if (formats[fmt].number_len)
        card->number = get_bit_field(reader, formats[fmt].number_start, formats[fmt].number_len);
    if (formats[fmt].cardholder_len)
        card->cardholder = get_bit_field(reader, formats[fmt].cardholder_start, formats[fmt].cardholder_len);

    return ESP_OK;
}

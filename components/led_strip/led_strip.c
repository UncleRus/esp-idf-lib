/**
 * @file led_strip.c
 *
 * RMT-based ESP-IDF driver for WS2812B/SK6812/APA106 LED strips
 *
 * Copyright (C) 2020 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "led_strip.h"
#include <esp_log.h>
#include <esp_attr.h>
#include <string.h>
#include <esp_idf_lib_helpers.h>

#if HELPER_TARGET_IS_ESP8266
#error led_strip is not supported on ESP8266
#endif

#ifndef RMT_DEFAULT_CONFIG_TX
#define RMT_DEFAULT_CONFIG_TX(gpio, channel_id)      \
    {                                                \
        .rmt_mode = RMT_MODE_TX,                     \
        .channel = channel_id,                       \
        .gpio_num = gpio,                            \
        .clk_div = 80,                               \
        .mem_block_num = 1,                          \
        .tx_config = {                               \
            .carrier_freq_hz = 38000,                \
            .carrier_level = RMT_CARRIER_LEVEL_HIGH, \
            .idle_level = RMT_IDLE_LEVEL_LOW,        \
            .carrier_duty_percent = 33,              \
            .carrier_en = false,                     \
            .loop_en = false,                        \
            .idle_output_en = true,                  \
        }                                            \
    }
#endif

static const char *TAG = "led_strip";

#define COLOR_SIZE 3
#define LED_STRIP_RMT_CLK_DIV 2

#define WS2812_T0H_NS   350
#define WS2812_T0L_NS   1000
#define WS2812_T1H_NS   1000
#define WS2812_T1L_NS   350

#define SK6812_T0H_NS   300
#define SK6812_T0L_NS   900
#define SK6812_T1H_NS   600
#define SK6812_T1L_NS   600

#define APA106_T0H_NS   350
#define APA106_T0L_NS   1360
#define APA106_T1H_NS   1360
#define APA106_T1L_NS   350

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static rmt_item32_t ws2812_bit0 = { 0 };
static rmt_item32_t ws2812_bit1 = { 0 };
static rmt_item32_t sk6812_bit0 = { 0 };
static rmt_item32_t sk6812_bit1 = { 0 };
static rmt_item32_t apa106_bit0 = { 0 };
static rmt_item32_t apa106_bit1 = { 0 };

static void IRAM_ATTR _rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
                                   size_t wanted_num, size_t *translated_size, size_t *item_num,
                                   const rmt_item32_t *bit0, const rmt_item32_t *bit1)
{
    if (!src || !dest)
    {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num)
    {
        for (int i = 0; i < 8; i++)
        {
            // MSB first
            pdest->val = *psrc & (1 << (7 - i)) ? bit1->val : bit0->val;
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

static void IRAM_ATTR ws2812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    _rmt_adapter(src, dest, src_size, wanted_num, translated_size, item_num, &ws2812_bit0, &ws2812_bit1);
}

static void IRAM_ATTR sk6812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    _rmt_adapter(src, dest, src_size, wanted_num, translated_size, item_num, &sk6812_bit0, &sk6812_bit1);
}

static void IRAM_ATTR apa106_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    _rmt_adapter(src, dest, src_size, wanted_num, translated_size, item_num, &apa106_bit0, &apa106_bit1);
}

///////////////////////////////////////////////////////////////////////////////

void led_strip_install()
{
    float ratio = (float)(APB_CLK_FREQ / LED_STRIP_RMT_CLK_DIV) / 1e09;

    ws2812_bit0.duration0 = ratio * WS2812_T0H_NS;
    ws2812_bit0.level0 = 1;
    ws2812_bit0.duration1 = ratio * WS2812_T0L_NS;
    ws2812_bit0.level1 = 0;
    ws2812_bit1.duration0 = ratio * WS2812_T1H_NS;
    ws2812_bit1.level0 = 1;
    ws2812_bit1.duration1 = ratio * WS2812_T1L_NS;
    ws2812_bit1.level1 = 0;

    sk6812_bit0.duration0 = ratio * SK6812_T0H_NS;
    sk6812_bit0.level0 = 1;
    sk6812_bit0.duration1 = ratio * SK6812_T0L_NS;
    sk6812_bit0.level1 = 0;
    sk6812_bit1.duration0 = ratio * SK6812_T1H_NS;
    sk6812_bit1.level0 = 1;
    sk6812_bit1.duration1 = ratio * SK6812_T1L_NS;
    sk6812_bit1.level1 = 0;

    apa106_bit0.duration0 = ratio * APA106_T0H_NS;
    apa106_bit0.level0 = 1;
    apa106_bit0.duration1 = ratio * APA106_T0L_NS;
    apa106_bit0.level1 = 0;
    apa106_bit1.duration0 = ratio * APA106_T1H_NS;
    apa106_bit1.level0 = 1;
    apa106_bit1.duration1 = ratio * APA106_T1L_NS;
    apa106_bit1.level1 = 0;
}

esp_err_t led_strip_init(led_strip_t *strip)
{
    CHECK_ARG(strip && strip->length > 0);

    strip->buf = malloc(strip->length * (COLOR_SIZE + (strip->is_rgbw > 0)));
    if (!strip->buf)
    {
        ESP_LOGE(TAG, "Not enough memory");
        return ESP_ERR_NO_MEM;
    }
    memset(strip->buf, 0, strip->length * (COLOR_SIZE + (strip->is_rgbw > 0)));

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(strip->gpio, strip->channel);
    config.clk_div = LED_STRIP_RMT_CLK_DIV;

    CHECK(rmt_config(&config));
    CHECK(rmt_driver_install(config.channel, 0, 0));

    sample_to_rmt_t f = NULL;
    switch (strip->type)
    {
        case LED_STRIP_WS2812:
            f = ws2812_rmt_adapter;
            break;
        case LED_STRIP_SK6812:
            f = sk6812_rmt_adapter;
            break;
        case LED_STRIP_APA106:
            f = apa106_rmt_adapter;
            break;
        default:
            ESP_LOGE(TAG, "Unknown strip type %d", strip->type);
            return ESP_ERR_NOT_SUPPORTED;
    }
    CHECK(rmt_translator_init(config.channel, f));

    return ESP_OK;
}

esp_err_t led_strip_free(led_strip_t *strip)
{
    CHECK_ARG(strip && strip->buf);
    free(strip->buf);

    CHECK(rmt_driver_uninstall(strip->channel));

    return ESP_OK;
}

esp_err_t led_strip_flush(led_strip_t *strip)
{
    CHECK_ARG(strip && strip->buf);

    CHECK(rmt_wait_tx_done(strip->channel, pdMS_TO_TICKS(CONFIG_LED_STRIP_FLUSH_TIMEOUT)));
    return rmt_write_sample(strip->channel, strip->buf,
                            strip->length * (COLOR_SIZE + (strip->is_rgbw > 0)), false);
}

bool led_strip_busy(led_strip_t *strip)
{
    if (!strip) return false;
    return rmt_wait_tx_done(strip->channel, 0) == ESP_ERR_TIMEOUT;
}

esp_err_t led_strip_wait(led_strip_t *strip, TickType_t timeout)
{
    CHECK_ARG(strip);

    return rmt_wait_tx_done(strip->channel, timeout);
}

esp_err_t led_strip_set_pixel(led_strip_t *strip, size_t num, rgb_t color)
{
    CHECK_ARG(strip && strip->buf && num <= strip->length);
    switch (strip->type)
    {
        case LED_STRIP_WS2812:
        case LED_STRIP_SK6812:
            // GRB
            ((uint8_t *)strip->buf)[num * COLOR_SIZE] = color.g;
            ((uint8_t *)strip->buf)[num * COLOR_SIZE + 1] = color.r;
            ((uint8_t *)strip->buf)[num * COLOR_SIZE + 2] = color.b;
            if (strip->is_rgbw)
                ((uint8_t *)strip->buf)[num * COLOR_SIZE + 3] = color.w;
            break;
        case LED_STRIP_APA106:
            // RGB
            ((uint8_t *)strip->buf)[num * COLOR_SIZE] = color.r;
            ((uint8_t *)strip->buf)[num * COLOR_SIZE + 1] = color.g;
            ((uint8_t *)strip->buf)[num * COLOR_SIZE + 2] = color.b;
            if (strip->is_rgbw)
                ((uint8_t *)strip->buf)[num * COLOR_SIZE + 3] = color.w;
            break;
        default:
            ESP_LOGE(TAG, "Unknown strip type %d", strip->type);
            return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

esp_err_t led_strip_set_pixels(led_strip_t *strip, size_t start, size_t len, rgb_t *data)
{
    CHECK_ARG(strip && strip->buf && len && start + len <= strip->length);
    switch (strip->type)
    {
        case LED_STRIP_WS2812:
        case LED_STRIP_SK6812:
            // GRB
            for (size_t i = 0; i < len; i++, data++)
            {
                ((uint8_t *)strip->buf)[(start + i) * COLOR_SIZE] = data->g;
                ((uint8_t *)strip->buf)[(start + i) * COLOR_SIZE + 1] = data->r;
                ((uint8_t *)strip->buf)[(start + i) * COLOR_SIZE + 2] = data->b;
                if (strip->is_rgbw)
                    ((uint8_t *)strip->buf)[(start + i) * COLOR_SIZE + 3] = data->w;
            }
            break;
        case LED_STRIP_APA106:
            // RGB, direct copy
            memcpy(strip->buf, data, len * (COLOR_SIZE + (strip->is_rgbw > 0)));
            break;
        default:
            ESP_LOGE(TAG, "Unknown strip type %d", strip->type);
            return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

esp_err_t led_strip_fill(led_strip_t *strip, size_t start, size_t len, rgb_t color)
{
    CHECK_ARG(strip && strip->buf && len && start + len <= strip->length);

    for (size_t i = start; i < len; i++)
        CHECK(led_strip_set_pixel(strip, i, color));
    return ESP_OK;
}

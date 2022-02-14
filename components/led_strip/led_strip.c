/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file led_strip.c
 *
 * RMT-based ESP-IDF driver for WS2812B/SK6812/APA106 LED strips
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "led_strip.h"
#include <esp_log.h>
#include <esp_attr.h>
#include <stdlib.h>
#if CONFIG_IDF_TARGET_ESP32
#include <esp32/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32S2
#include <esp32s2/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32S3
#include <esp32s3/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32C3
#include <esp32c3/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32H2
#include <esp32h2/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32C2
#include <esp32c2/rom/ets_sys.h>
#endif
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

#define LED_STRIP_RMT_CLK_DIV 2

#define WS2812_T0H_NS   400
#define WS2812_T0L_NS   1000
#define WS2812_T1H_NS   1000
#define WS2812_T1L_NS   400

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

#define COLOR_SIZE(strip) (3 + ((strip)->is_rgbw != 0))

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
#ifdef LED_STRIP_BRIGHTNESS
    led_strip_t *strip;
    esp_err_t r = rmt_translator_get_context(item_num, (void **)&strip);
    uint8_t brightness = r == ESP_OK ? strip->brightness : 255;
#endif
    while (size < src_size && num < wanted_num)
    {
#ifdef LED_STRIP_BRIGHTNESS
        uint8_t b = brightness != 255 ? scale8_video(*psrc, brightness) : *psrc;
#else
        uint8_t b = *psrc;
#endif
        for (int i = 0; i < 8; i++)
        {
            // MSB first
            pdest->val = b & (1 << (7 - i)) ? bit1->val : bit0->val;
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
    CHECK_ARG(strip && strip->length > 0 && strip->type < LED_STRIP_TYPE_MAX);

    strip->buf = calloc(strip->length, COLOR_SIZE(strip));
    if (!strip->buf)
    {
        ESP_LOGE(TAG, "Not enough memory");
        return ESP_ERR_NO_MEM;
    }

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
            break;
    }
    CHECK(rmt_translator_init(config.channel, f));
#ifdef LED_STRIP_BRIGHTNESS
    // No support for translator context prior to ESP-IDF 4.3
    CHECK(rmt_translator_set_context(config.channel, strip));
#endif

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
    ets_delay_us(CONFIG_LED_STRIP_PAUSE_LENGTH);
    return rmt_write_sample(strip->channel, strip->buf,
                            strip->length * COLOR_SIZE(strip), false);
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
    size_t idx = num * COLOR_SIZE(strip);
    switch (strip->type)
    {
        case LED_STRIP_WS2812:
        case LED_STRIP_SK6812:
            // GRB
            strip->buf[idx] = color.g;
            strip->buf[idx + 1] = color.r;
            strip->buf[idx + 2] = color.b;
            if (strip->is_rgbw)
                strip->buf[idx + 3] = rgb_luma(color);
            break;
        case LED_STRIP_APA106:
            // RGB
            strip->buf[idx] = color.r;
            strip->buf[idx + 1] = color.g;
            strip->buf[idx + 2] = color.b;
            if (strip->is_rgbw)
                strip->buf[idx + 3] = rgb_luma(color);
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
    for (size_t i = 0; i < len; i++)
        CHECK(led_strip_set_pixel(strip, i + start, data[i]));
    return ESP_OK;
}

esp_err_t led_strip_fill(led_strip_t *strip, size_t start, size_t len, rgb_t color)
{
    CHECK_ARG(strip && strip->buf && len && start + len <= strip->length);

    for (size_t i = start; i < start + len; i++)
        CHECK(led_strip_set_pixel(strip, i, color));
    return ESP_OK;
}

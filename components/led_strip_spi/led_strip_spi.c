/**
 * @file led_strip.c
 *
 * SPI-based ESP-IDF driver for SK9822 LED strips
 *
 * Copyright (C) 2020 Ruslan V. Uss <https://github.com/UncleRus>
 *               2021 Tomoyuki Sakurai <y@trombik.org>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <driver/spi_master.h>
#include <esp_idf_lib_helpers.h>
#include "led_strip_spi.h"

#if defined(CONFIG_LED_STRIP_SPI_USING_SK9822)
#include "led_strip_spi_sk9822.h"
#endif

#if HELPER_TARGET_IS_ESP8266
#error led_strip_spi is not (yet) supported on ESP8266
#endif

static const char *TAG = "led_strip_spi";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

void led_strip_spi_install()
{
    /* NOOP, for compatibility */
}

esp_err_t led_strip_spi_init(led_strip_spi_t *strip)
{
    esp_err_t r = ESP_FAIL;

    strip->buf = heap_caps_malloc(LED_STRIP_SPI_BUFFER_SIZE(strip->length), MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
    if (strip->buf == NULL) {
        ESP_LOGE(TAG, "heap_caps_malloc()");
        r = ESP_ERR_NO_MEM;
        goto fail;
    }
    memset(strip->buf, 0, LED_STRIP_SPI_BUFFER_SIZE(strip->length));

    /* XXX length is in bit */
    strip->transaction.length = LED_STRIP_SPI_BUFFER_SIZE(strip->length) * 8;

    /* set mandatory bits in all LED frames */
    for (int i = 1; i <= strip->length; i++) {
        ((uint8_t *)strip->buf)[i * 4] = LED_STRIP_SPI_FRAME_SK9822_LED_MSB3;
    }
    ESP_LOGD(TAG, "SPI buffer initialized");

    CHECK(spi_bus_initialize(strip->host_device, &strip->bus_config, strip->dma_chan));
    ESP_LOGD(TAG, "SPI bus initialized");

    CHECK(spi_bus_add_device(strip->host_device, &strip->device_interface_config, &strip->device_handle));
    ESP_LOGI(TAG, "LED strip initialized");

    /* turn off all LEDs */
    r = led_strip_spi_flush(strip);
fail:
    return r;
}

esp_err_t led_strip_spi_free(led_strip_spi_t *strip)
{
    CHECK_ARG(strip);

    free(strip->buf);
    return ESP_OK;
}

esp_err_t led_strip_spi_flush(led_strip_spi_t*strip)
{
    esp_err_t r = ESP_FAIL;
    spi_transaction_t* t;

    CHECK_ARG(strip);
    if (!strip->transaction.tx_buffer) {
        strip->transaction.tx_buffer = strip->buf;
    }
    strip->transaction.tx_buffer = strip->buf;
    r = spi_device_queue_trans(strip->device_handle, &strip->transaction, portMAX_DELAY);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_queue_trans(): %s", esp_err_to_name(r));
        goto fail;
    }
    r = spi_device_get_trans_result(strip->device_handle, &t, portMAX_DELAY);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_get_trans_result(): %s", esp_err_to_name(r));
        goto fail;
    }
    r = ESP_OK;
fail:
    return r;
}

bool led_strip_spi_busy(led_strip_spi_t*strip)
{

    /* XXX FIXME */
    ESP_LOGW(TAG, "led_strip_spi_busy() not implemented");
    if (!strip) return false;
    return false;
}

esp_err_t led_strip_spi_wait(led_strip_spi_t*strip, TickType_t timeout)
{
    CHECK_ARG(strip);

    /* XXX FIXME */
    ESP_LOGW(TAG, "led_strip_spi_wait() not implemented");
    return false;
}

esp_err_t led_strip_spi_set_pixels(led_strip_spi_t*strip, size_t start, size_t len, rgb_t *data)
{
    /* XXX FIXME */
    ESP_LOGW(TAG, "led_strip_spi_set_pixels() not implemented");
    return ESP_OK;
}

esp_err_t led_strip_spi_fill(led_strip_spi_t*strip, size_t start, size_t len, rgb_t color)
{
    CHECK_ARG(strip && len && start + len <= strip->length);

    for (size_t i = start; i < len; i++) {
        CHECK(led_strip_spi_set_pixel(strip, i, color));
    }
    return ESP_OK;
}

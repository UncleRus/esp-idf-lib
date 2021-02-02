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
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
#include <esp_idf_lib_helpers.h>
#include "led_strip_spi.h"

#if defined(CONFIG_LED_STRIP_SPI_USING_SK9822)
#include "led_strip_spi_sk9822.h"
#endif

#if HELPER_TARGET_IS_ESP32
#include <driver/spi_master.h>
#elif HELPER_TARGET_IS_ESP8266
#include <driver/spi.h>
#endif

static const char *TAG = "led_strip_spi";
static SemaphoreHandle_t mutex;

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define MUTEX_TIMEOUT   (CONFIG_LED_STRIP_SPI_MUTEX_TIMEOUT_MS / portTICK_RATE_MS)

esp_err_t led_strip_spi_install()
{
    esp_err_t err;

    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "xSemaphoreCreateMutex(): faild");
        goto fail;
    }
fail:
    return err;
}

#if HELPER_TARGET_IS_ESP32
static esp_err_t led_strip_spi_init_esp32(led_strip_spi_t *strip)
{
    esp_err_t err = ESP_FAIL;

    if (xSemaphoreTake(mutex, MUTEX_TIMEOUT) != pdTRUE) {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "xSemaphoreTake(): timeout");
        goto fail_without_give;
    }

    strip->buf = heap_caps_malloc(LED_STRIP_SPI_BUFFER_SIZE(strip->length), MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
    if (strip->buf == NULL) {
        ESP_LOGE(TAG, "heap_caps_malloc()");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    memset(strip->buf, 0, LED_STRIP_SPI_BUFFER_SIZE(strip->length));

    /* XXX length is in bit */
    strip->transaction.length = LED_STRIP_SPI_BUFFER_SIZE(strip->length) * 8;

#if CONFIG_LED_STRIP_SPI_USING_SK9822
    err = led_strip_spi_sk9822_buf_init(strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_spi_sk9822_buf_init(): %s", esp_err_to_name(err));
        goto fail;
    }
#endif
    ESP_LOGD(TAG, "SPI buffer initialized");

    err = spi_bus_initialize(strip->host_device, &strip->bus_config, strip->dma_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize(): %s", esp_err_to_name(err));
        goto fail;
    }
    ESP_LOGD(TAG, "SPI bus initialized");

    err = spi_bus_add_device(strip->host_device, &strip->device_interface_config, &strip->device_handle);
    if (err != ESP_OK) {
        ESP_LOG(TAG, "spi_bus_add_device(): %s", esp_err_to_name(err));
        goto fail;
    }
    ESP_LOGI(TAG, "LED strip initialized");
fail:
    if (xSemaphoreGive(mutex) != pdTRUE) {
        ESP_LOGE(TAG, "xSemaphoreGive(): failed");
    }
fail_without_give:
    return err;
}
#endif

#if HELPER_TARGET_IS_ESP8266
static esp_err_t led_strip_spi_init_esp8266(led_strip_spi_t *strip)
{
    esp_err_t err;
    spi_config_t spi_config;

    if (xSemaphoreTake(mutex, MUTEX_TIMEOUT) != pdTRUE) {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "xSemaphoreTake(): timeout");
        goto fail_without_give;
    }

    strip->buf = malloc(LED_STRIP_SPI_BUFFER_SIZE(strip->length));
    if (strip->buf == NULL) {
        ESP_LOGE(TAG, "malloc()");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    memset(strip->buf, 0, LED_STRIP_SPI_BUFFER_SIZE(strip->length));

#if CONFIG_LED_STRIP_SPI_USING_SK9822
    err = led_strip_spi_sk9822_buf_init(strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_spi_sk9822_buf_init(): %s", esp_err_to_name(err));
        goto fail;
    }
#endif
    ESP_LOGI(TAG, "SPI buffer initialized");

    spi_config.interface = strip->bus_config;
    spi_config.mode = SPI_MASTER_MODE;
    spi_config.clk_div = strip->clk_div;
    spi_config.event_cb = NULL;
    err = spi_init(strip->host_device, &spi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_init(): %s", esp_err_to_name(err));
        goto fail;
    }
    ESP_LOGI(TAG, "SPI bus initialized");
    ESP_LOGI(TAG, "LED strip initialized");
fail:
    if (xSemaphoreGive(mutex) != pdTRUE) {
        ESP_LOGE(TAG, "xSemaphoreGive(): failed");
    }
fail_without_give:
    return err;
}
#endif

esp_err_t led_strip_spi_init(led_strip_spi_t *strip)
{
#if HELPER_TARGET_IS_ESP32
    return led_strip_spi_init_esp32(strip);
#elif HELPER_TARGET_IS_ESP8266
    return led_strip_spi_init_esp8266(strip);
#else
#error "Unknown target"
#endif
}

esp_err_t led_strip_spi_free(led_strip_spi_t *strip)
{
    CHECK_ARG(strip);

    free(strip->buf);
    return ESP_OK;
}

#if HELPER_TARGET_IS_ESP32
static esp_err_t led_strip_spi_flush_esp32(led_strip_spi_t *strip)
{
    esp_err_t err = ESP_FAIL;
    spi_transaction_t* t;

    CHECK_ARG(strip);
    if (!strip->transaction.tx_buffer) {
        strip->transaction.tx_buffer = strip->buf;
    }
    strip->transaction.tx_buffer = strip->buf;
    err = spi_device_queue_trans(strip->device_handle, &strip->transaction, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_queue_trans(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = spi_device_get_trans_result(strip->device_handle, &t, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_get_trans_result(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = ESP_OK;
fail:
    return err;
}
#endif

#if HELPER_TARGET_IS_ESP8266
static esp_err_t led_strip_spi_flush_esp8266(led_strip_spi_t *strip)
{
    esp_err_t err = ESP_FAIL;
    spi_trans_t trans = {0};

    CHECK_ARG(strip);

    if (xSemaphoreTake(mutex, MUTEX_TIMEOUT) != pdTRUE) {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "xSemaphoreTake(): timeout");
        goto fail_without_give;
    }

    trans.bits.mosi = LED_STRIP_SPI_BUFFER_SIZE(strip->length) * 8;
    trans.mosi = strip->buf;
    err = spi_trans(HSPI_HOST, &trans);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_trans(): %s", esp_err_to_name(err));
        goto fail;
    }
fail:
    if (xSemaphoreGive(mutex) != pdTRUE) {
        ESP_LOGE(TAG, "xSemaphoreGive(): failed");
    }
fail_without_give:
    return err;
}
#endif
esp_err_t led_strip_spi_flush(led_strip_spi_t*strip)
{
#if HELPER_TARGET_IS_ESP32
    return led_strip_spi_flush_esp32(strip);
#elif HELPER_TARGET_IS_ESP8266
    return led_strip_spi_flush_esp8266(strip);
#else
#error "Unknown target"
#endif
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

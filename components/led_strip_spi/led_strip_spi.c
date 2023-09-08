/*
 * MIT License
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *               2021 Tomoyuki Sakurai <y@rombik.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
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
 * SPI-based ESP-IDF driver for SK9822 LED strips
 *
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
#define MUTEX_TIMEOUT   (CONFIG_LED_STRIP_SPI_MUTEX_TIMEOUT_MS / portTICK_PERIOD_MS)

esp_err_t led_strip_spi_install()
{
    esp_err_t err;

    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "xSemaphoreCreateMutex(): failed");
        goto fail;
    }
    err = ESP_OK;
fail:
    return err;
}

#if HELPER_TARGET_IS_ESP32
static esp_err_t led_strip_spi_init_esp32(led_strip_spi_t *strip)
{
    CHECK_ARG(strip);

    esp_err_t err = ESP_FAIL;
    spi_bus_config_t bus_config = {
        .mosi_io_num = strip->mosi_io_num,
        .sclk_io_num = strip->sclk_io_num,
        .miso_io_num = -1,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
#endif
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .max_transfer_sz = strip->max_transfer_sz,
    };
    spi_device_interface_config_t device_interface_config = {
        .clock_speed_hz = strip->clock_speed_hz,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = strip->queue_size,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
    };

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

    /* type-specific initialization  */
#if CONFIG_LED_STRIP_SPI_USING_SK9822
    err = led_strip_spi_sk9822_buf_init(strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_spi_sk9822_buf_init(): %s", esp_err_to_name(err));
        goto fail;
    }
#endif
    ESP_LOGD(TAG, "SPI buffer initialized");

    err = spi_bus_initialize(strip->host_device, &bus_config, strip->dma_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize(): %s", esp_err_to_name(err));
        goto fail;
    }
    ESP_LOGD(TAG, "SPI bus initialized");

    err = spi_bus_add_device(strip->host_device, &device_interface_config, &strip->device_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device(): %s", esp_err_to_name(err));
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
    spi_interface_t interface_config = {

        /* SPI mode 3, CPOL = 1, CPHA = 1 */
        .cpol = 1,
        .cpha = 1,
        .bit_tx_order = 0,
        .bit_rx_order = 0,
        .byte_tx_order = 0,
        .byte_rx_order = 0,
        .mosi_en = 1,
        .miso_en = 0,
        .cs_en = 0,
        .reserved9 = 23,
    };

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
    spi_config.interface = interface_config;
    spi_config.mode = SPI_MASTER_MODE;
    spi_config.clk_div = strip->clk_div;
    spi_config.event_cb = NULL;

    err = spi_init(HSPI_HOST, &spi_config);
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

#define ESP8266_SPI_MAX_DATA_LENGTH 64 // in bytes

static esp_err_t led_strip_spi_flush_esp8266(led_strip_spi_t *strip)
{
    esp_err_t err = ESP_FAIL;
    spi_trans_t trans = {0};
    int mosi_buffer_block_size, mosi_buffer_block_size_mod;

    CHECK_ARG(strip);

    /* XXX send ESP8266_SPI_MAX_DATA_LENGTH bytes data at a time. the
     * documentation does not mention the limitation, but the SPI master
     * driver complains:
     * "spi: spi_master_trans(454): spi mosi must be shorter than 512 bits" */
    mosi_buffer_block_size = LED_STRIP_SPI_BUFFER_SIZE(strip->length) / ESP8266_SPI_MAX_DATA_LENGTH;
    mosi_buffer_block_size_mod = LED_STRIP_SPI_BUFFER_SIZE(strip->length) % ESP8266_SPI_MAX_DATA_LENGTH;

    if (xSemaphoreTake(mutex, MUTEX_TIMEOUT) != pdTRUE) {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "xSemaphoreTake(): timeout");
        goto fail_without_give;
    }

    for (int i = 0; i < mosi_buffer_block_size; i++) {
        trans.bits.mosi = ESP8266_SPI_MAX_DATA_LENGTH * 8; // bits, not bytes
        trans.mosi = strip->buf + ESP8266_SPI_MAX_DATA_LENGTH * i;
        err = spi_trans(HSPI_HOST, &trans);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_trans(): %s", esp_err_to_name(err));
            goto fail;
        }
    }
    if (mosi_buffer_block_size_mod > 0) {
        trans.bits.mosi = mosi_buffer_block_size_mod * 8; // bits, not bytes
        trans.mosi = strip->buf + ESP8266_SPI_MAX_DATA_LENGTH * mosi_buffer_block_size;
        err = spi_trans(HSPI_HOST, &trans);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_trans(): %s", esp_err_to_name(err));
            goto fail;
        }
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

esp_err_t led_strip_spi_set_pixel(led_strip_spi_t *strip, const int index, const rgb_t color)
{
    return led_strip_spi_set_pixel_brightness(strip, index, color, LED_STRIP_SPI_MAX_BRIGHTNESS);
}

esp_err_t led_strip_spi_set_pixels(led_strip_spi_t*strip, const int start, size_t len, const rgb_t data)
{
    return led_strip_spi_set_pixels_brightness(strip, start, len, data, LED_STRIP_SPI_MAX_BRIGHTNESS);
}

esp_err_t led_strip_spi_fill(led_strip_spi_t*strip, size_t start, size_t len, rgb_t color)
{
    return led_strip_spi_fill_brightness(strip, start, len, color, LED_STRIP_SPI_MAX_BRIGHTNESS);
}

esp_err_t led_strip_spi_set_pixel_brightness(led_strip_spi_t *strip, const int index, const rgb_t color, const uint8_t brightness)
{
#if CONFIG_LED_STRIP_SPI_USING_SK9822
    return led_strip_spi_set_pixel_sk9822(strip, index, color, brightness);
#endif
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t led_strip_spi_set_pixels_brightness(led_strip_spi_t*strip, const int start, size_t len, const rgb_t data, const uint8_t brightness)
{
    esp_err_t err = ESP_FAIL;

    for (int i = 0; i < len; i++) {
        err = led_strip_spi_set_pixel_brightness(strip, start + i, data, brightness);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "led_strip_spi_set_pixel(): %s", esp_err_to_name(err));
            goto fail;
        }
    }
fail:
    return err;
}

esp_err_t led_strip_spi_fill_brightness(led_strip_spi_t*strip, size_t start, size_t len, rgb_t color, const uint8_t brightness)
{
    CHECK_ARG(strip && len && start + len <= strip->length);

    for (size_t i = start; i < len; i++) {
        CHECK(led_strip_spi_set_pixel_brightness(strip, i, color, brightness));
    }
    return ESP_OK;
}

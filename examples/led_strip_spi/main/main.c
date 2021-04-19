#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <led_strip_spi.h>
#include "effect.h"

/*
 * For ESP32 and ESP32S2
 * SCLK GPIO14
 * MOSI GPIO13
 *
 * For ESP8266
 * SCLK GPIO14 (D5 on NodeMCU devkit)
 * MOSI GPIO13 (D7 on NodeMCU devkit)
 */

#define N_PIXEL CONFIG_EXAMPLE_N_PIXEL
#define TAG "example_led_strip_spi"

static esp_err_t rainbow(led_strip_spi_t *strip)
{
    static uint8_t pos = 0;
    esp_err_t err = ESP_FAIL;
    rgb_t color;

    color = led_effect_color_wheel_rgb(pos);
    ESP_LOGI(TAG, "r: 0x%02x g: 0x%02x b: 0x%02x", color.r, color.g, color.b);

    if ((err = led_strip_spi_fill(strip, 0, strip->length, color)) != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_spi_fill(): %s", esp_err_to_name(err));
        goto fail;
    }
    pos += 1;
fail:
    return err;
}

static esp_err_t rainbow_scroll(led_strip_spi_t *strip)
{
    static uint8_t pos = 0;
    const uint8_t offset = 0xff / N_PIXEL >= 1 ? 0xff / N_PIXEL : 1;

    esp_err_t err = ESP_FAIL;
    rgb_t color;

    for (int i = 0; i < N_PIXEL; i++) {
        color = led_effect_color_wheel_rgb(pos + offset * i);
        err = led_strip_spi_set_pixel(strip, i, color);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "led_strip_spi_set_pixel(): %s", esp_err_to_name(err));
            goto fail;
        }
    }
    ESP_LOGI(TAG, "pos: %d", pos);
    pos += 1;
fail:
    return err;

}

static esp_err_t simple_rgb(led_strip_spi_t *strip)
{
    static uint8_t counter = 0;
    uint32_t c;
    rgb_t color;
    esp_err_t err;

    c = 0x0000a0 << ((counter % 3) * 8);
    color.r = (c >> 16) & 0xff;
    color.g = (c >> 8)  & 0xff;
    color.b =  c        & 0xff;
    ESP_LOGI(TAG, "r: 0x%02x g: 0x%02x b: 0x%02x", color.r, color.g, color.b);

    if ((err = led_strip_spi_fill(strip, 0, strip->length, color)) != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_spi_fill(): %s", esp_err_to_name(err));
        goto fail;
    }
    counter += 1;
fail:
    return err;
}

void test(void *pvParameters)
{
    led_strip_spi_t strip = LED_STRIP_SPI_DEFAULT();
    strip.length = N_PIXEL;
#if HELPER_TARGET_IS_ESP32
    static spi_device_handle_t device_handle;
    strip.device_handle = device_handle;
    strip.max_transfer_sz = LED_STRIP_SPI_BUFFER_SIZE(N_PIXEL);
    strip.clock_speed_hz = 1000000 * 10; // 10Mhz
#endif
#if HELPER_TARGET_IS_ESP8266
    strip.clk_div = SPI_10MHz_DIV;
#endif

    ESP_LOGI(TAG, "Initializing LED strip");
    ESP_ERROR_CHECK(led_strip_spi_init(&strip));

    /* turn off all LEDs */
    ESP_ERROR_CHECK(led_strip_spi_flush(&strip));
    while (1) {

        /* rainbow */
        for (int i = 0; i < 255; i++) {
            ESP_ERROR_CHECK(rainbow(&strip));
            ESP_ERROR_CHECK(led_strip_spi_flush(&strip));

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        /* simple RGB */
        for (int i = 0; i < 10; i++) {
            ESP_ERROR_CHECK(simple_rgb(&strip));
            ESP_ERROR_CHECK(led_strip_spi_flush(&strip));

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        /* rainbow scroll */
        for (int i = 0; i < 2048; i++) {
            ESP_ERROR_CHECK(rainbow_scroll(&strip));
            ESP_ERROR_CHECK(led_strip_spi_flush(&strip));

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void app_main()
{
    esp_err_t err;
    err = led_strip_spi_install();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_spi_install(): %s", esp_err_to_name(err));
        goto fail;
    }
    if (xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(): failed");
        goto fail;
    }
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
fail:
    ESP_LOGE(TAG, "Test failed");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

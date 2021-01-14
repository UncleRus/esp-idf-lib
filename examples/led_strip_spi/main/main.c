#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <led_effect.h>
#include <led_strip_spi.h>

/*
 * SCLK GPIO14
 * MOSI GPIO13
 */

#define N_PIXEL CONFIG_EXAMPLE_N_PIXEL
#define TAG "example_led_strip_spi"

static esp_err_t rainbow(led_strip_spi_t *strip) {
    static uint8_t pos = 0;
    uint32_t c;
    esp_err_t err = ESP_FAIL;
    rgb_t color;

    c = led_effect_color_wheel(pos);
    color.r = (c >> 16) & 0xff;
    color.g = (c >> 8)  & 0xff;
    color.b =  c        & 0xff;
    ESP_LOGI(TAG, "r: 0x%02x g: 0x%02x b: 0x%02x", color.r, color.g, color.b);

    if ((err = led_strip_spi_fill(strip, 0, strip->length, color)) != ESP_OK) {
        goto fail;
    }
    pos += 1;
fail:
    return err;
}

void test(void *pvParameters)
{
    static spi_device_handle_t device_handle;
    led_strip_spi_t strip = LED_STRIP_SPI_DEFAULT();
    strip.device_handle = device_handle;
    strip.length = N_PIXEL;
    strip.bus_config.max_transfer_sz = LED_STRIP_SPI_BUFFER_SIZE(N_PIXEL);

    ESP_LOGI(TAG, "Initializing LED strip");
    ESP_ERROR_CHECK(led_strip_spi_init(&strip));

    while (1)
    {
        ESP_ERROR_CHECK(rainbow(&strip));
        ESP_ERROR_CHECK(led_strip_spi_flush(&strip));

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    led_strip_spi_install();
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}

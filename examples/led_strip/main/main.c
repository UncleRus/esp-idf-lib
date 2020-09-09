#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>

#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO 5
#define LED_CHANNEL RMT_CHANNEL_0
#define LED_STRIP_LEN 256

static const rgb_t colors[] = {
    { .raw = { 0xff, 0xff, 0xff } },
    { .raw = { 0x00, 0x00, 0xff } },
    { .raw = { 0x00, 0xff, 0x00 } },
    { .raw = { 0xff, 0x00, 0x00 } },
    { .raw = { 0x00, 0x00, 0x00 } },
};

#define COLORS_TOTAL (sizeof(colors) / sizeof(rgb_t))

void test(void *pvParameters)
{
    led_strip_t strip = {
        .type = LED_TYPE,
        .length = LED_STRIP_LEN,
        .gpio = LED_GPIO,
        .channel = LED_CHANNEL,
        .buf = NULL
    };

    ESP_ERROR_CHECK(led_strip_init(&strip));

    size_t c = 0;
    while (1)
    {
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[c]));
        ESP_ERROR_CHECK(led_strip_flush(&strip));

        vTaskDelay(pdMS_TO_TICKS(1000));

        if (++c >= COLORS_TOTAL)
            c = 0;
    }
}

void app_main()
{
    led_strip_install();
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}


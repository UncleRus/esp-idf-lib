#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>

#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO CONFIG_LED_STRIP_GPIO

static const rgb_t colors[] = {
    { .r = 0x0f, .g = 0x0f, .b = 0x0f },
    { .r = 0x00, .g = 0x00, .b = 0x2f },
    { .r = 0x00, .g = 0x2f, .b = 0x00 },
    { .r = 0x2f, .g = 0x00, .b = 0x00 },
    { .r = 0x00, .g = 0x00, .b = 0x00 },
};

#define COLORS_TOTAL (sizeof(colors) / sizeof(rgb_t))

void test(void *pvParameters)
{
    led_strip_t strip = {
        .type = LED_TYPE,
        .length = CONFIG_LED_STRIP_LEN,
        .gpio = LED_GPIO,
        .buf = NULL,
#ifdef LED_STRIP_BRIGHTNESS
        .brightness = 255,
#endif
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


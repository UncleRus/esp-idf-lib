#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds1302.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define CE_GPIO 5
#define IO_GPIO 4
#define SCLK_GPIO 0
#else
#define CE_GPIO 16
#define IO_GPIO 17
#define SCLK_GPIO 18
#endif

void ds1302_test(void *pvParameters)
{
    ds1302_t dev = {
        .ce_pin = CE_GPIO,
        .io_pin = IO_GPIO,
        .sclk_pin = SCLK_GPIO
    };

    ESP_ERROR_CHECK(ds1302_init(&dev));
    ESP_ERROR_CHECK(ds1302_set_write_protect(&dev, false));

    // setup datetime: 2018-04-11 00:52:10
    struct tm time = {
        .tm_year = 118, //year since 1900 (2018 - 1900)
        .tm_mon  = 3,  // 0-based
        .tm_mday = 11,
        .tm_hour = 0,
        .tm_min  = 52,
        .tm_sec  = 10
    };
    ESP_ERROR_CHECK(ds1302_set_time(&dev, &time));
    ESP_ERROR_CHECK(ds1302_start(&dev, true));

    while (1)
    {
        ds1302_get_time(&dev, &time);

        printf("%04d-%02d-%02d %02d:%02d:%02d\n", time.tm_year + 1900 /*Add 1900 for better readability*/, time.tm_mon + 1,
            time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(ds1302_test, "ds1302_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}


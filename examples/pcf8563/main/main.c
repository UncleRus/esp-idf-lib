#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pcf8563.h>
#include <string.h>
#include <esp_err.h>

void test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(pcf8563_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // setup datetime: 2020-04-03 12:35, Sunday
    struct tm time = {
        .tm_year = 120, // years since 1900
        .tm_mon  = 3,   // months since January
        .tm_mday = 3,
        .tm_hour = 12,
        .tm_min  = 35,
        .tm_sec  = 10,
        .tm_wday = 0    // days since Sunday
    };
    ESP_ERROR_CHECK(pcf8563_set_time(&dev, &time));

    while (1)
    {
        bool valid;
        esp_err_t r = pcf8563_get_time(&dev, &time, &valid);
        if (r == ESP_OK)
            printf("%04d-%02d-%02d %02d:%02d:%02d, %s\n", time.tm_year + 1900, time.tm_mon + 1,
                time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, valid ? "VALID" : "NOT VALID");
        else
            printf("Error %d: %s\n", r, esp_err_to_name(r));


        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}


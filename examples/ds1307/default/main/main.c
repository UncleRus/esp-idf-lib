#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds1307.h>
#include <string.h>

void ds1307_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds1307_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // setup datetime: 2018-04-11 00:52:10
    struct tm time = {
        .tm_year = 118, //since 1900 (2018 - 1900)
        .tm_mon  = 3,  // 0-based
        .tm_mday = 11,
        .tm_hour = 0,
        .tm_min  = 52,
        .tm_sec  = 10
    };
    ESP_ERROR_CHECK(ds1307_set_time(&dev, &time));

    while (1)
    {
        ds1307_get_time(&dev, &time);

        printf("%04d-%02d-%02d %02d:%02d:%02d\n", time.tm_year + 1900 /*Add 1900 for better readability*/, time.tm_mon + 1,
            time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(ds1307_test, "ds1307_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}


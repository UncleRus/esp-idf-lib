#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bmp180.h>
#include <string.h>

#define SDA_GPIO 16
#define SCL_GPIO 17

void bmp180_test(void *pvParameters)
{
    bmp180_dev_t dev;

    memset(&dev, 0, sizeof(bmp180_dev_t)); // Zero descriptor

    esp_err_t res;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (bmp180_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while ((res = bmp180_init(&dev)) != ESP_OK)
    {
        printf("Could not init BMP180, err: %d\n", res);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (1)
    {
        float temp;
        uint32_t pressure;

        printf("Current core: %d\n", xPortGetCoreID());

        res = bmp180_measure(&dev, &temp, &pressure, BMP180_MODE_STANDARD);
        if (res != ESP_OK)
            printf("Could not measure: %d\n", res);
        else
            printf("Temperature: %.2f degrees Celsius; Pressure: %d MPa\n", temp, pressure);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(bmp180_test, "bmp180_test", configMINIMAL_STACK_SIZE * 15, NULL, 5, NULL, APP_CPU_NUM);
}


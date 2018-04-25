#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <tsl2561.h>

#define SDA_GPIO 16
#define SCL_GPIO 17
#define ADDR TSL2561_I2C_ADDR_FLOAT

void tsl2561_test(void *pvParamters)
{
    tsl2561_t dev;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (tsl2561_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    esp_err_t res;
    while ((res = tsl2561_init(&dev)) != ESP_OK)
    {
        printf("Could not init TSL2561, err: %d\n", res);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    printf("Found TSL2561 in package %s\n", dev.package_type == TSL2561_PACKAGE_CS ? "CS" : "T/FN/CL");

    uint32_t lux;
    while (1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        if ((res = tsl2561_read_lux(&dev, &lux)) != ESP_OK)
            printf("Could not read lux value: %d\n", res);
        else
            printf("Lux: %u\n", lux);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(tsl2561_test, "tsl2561_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


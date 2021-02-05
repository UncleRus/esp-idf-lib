#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <tsl2561.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 18
#define SCL_GPIO 19
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define ADDR TSL2561_I2C_ADDR_FLOAT

void tsl2561_test(void *pvParameters)
{
    tsl2561_t dev;
    memset(&dev, 0, sizeof(tsl2561_t));

    ESP_ERROR_CHECK(tsl2561_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(tsl2561_init(&dev));

    printf("Found TSL2561 in package %s\n", dev.package_type == TSL2561_PACKAGE_CS ? "CS" : "T/FN/CL");

    uint32_t lux;
    esp_err_t res;
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
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(tsl2561_test, "tsl2561_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <tsl2591.h>
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

void tsl2591_test(void *pvParameters)
{
    tsl2591_t dev;
    memset(&dev, 0, sizeof(tsl2591_t));

    ESP_ERROR_CHECK(tsl2591_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(tsl2591_init(&dev));

    float lux;
    esp_err_t res;
    while (1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        if ((res = tsl2591_get_lux(&dev, &lux)) != ESP_OK)
            printf("Could not read lux value: %d\n", res);
        else
            printf("Lux: %f\n", lux);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(tsl2591_test, "tsl2591_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


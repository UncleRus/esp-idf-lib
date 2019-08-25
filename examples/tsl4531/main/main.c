#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <tsl4531.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

void tsl4531_test(void *pvParamters)
{
    tsl4531_t dev;
    memset(&dev, 0, sizeof(tsl4531_t));

    ESP_ERROR_CHECK(tsl4531_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));

    ESP_ERROR_CHECK(tsl4531_init(&dev));
    ESP_ERROR_CHECK(tsl4531_config(&dev, TSL4531_INTEGRATION_400MS, true));

    uint16_t lux;
    esp_err_t res;
    while (1)
    {
        if ((res = tsl4531_read_lux(&dev, &lux)) != ESP_OK)
            printf("Could not read lux value: %d\n", res);
        else
            printf("Lux: %u\n", lux);

        // 0.05 second delay
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(tsl4531_test, "tsl4531_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

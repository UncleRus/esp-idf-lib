#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <tsl4531.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void tsl4531_test(void *pvParameters)
{
    tsl4531_t dev = { 0 };

    ESP_ERROR_CHECK(tsl4531_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

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

        // 0.5 second delay
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(tsl4531_test, "tsl4531_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

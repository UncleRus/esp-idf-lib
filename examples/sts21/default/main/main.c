#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <sts21.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "sts21-example";

void task(void *pvParameters)
{
    sts21_t dev = { 0 };

    ESP_ERROR_CHECK(sts21_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sts21_init(&dev));

    float temperature;
    while (1)
    {
        esp_err_t res = sts21_measure(&dev, &temperature);
        if (res == ESP_OK)
            ESP_LOGI(TAG, "Temperature: %.1f°C", temperature);
        else
            ESP_LOGE(TAG, "Error reading data: %d (%s)", res, esp_err_to_name(res));

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


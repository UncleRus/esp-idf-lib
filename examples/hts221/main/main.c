#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hts221.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 2
#define SCL_GPIO 3
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "hts221-example";

void task(void *pvParameters)
{
    hts221_t dev = { 0 };

    // Initialize HTS221 descriptor and setup HTS221 to default values according to datasheet.
    ESP_ERROR_CHECK(hts221_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(hts221_init(&dev));
    // Set data rate to 1Hz
    ESP_ERROR_CHECK(hts221_set_data_rate(&dev, HTS221_1HZ));

    float temperature, humidity;

    while (1)
    {
        esp_err_t res = hts221_get_data(&dev, &temperature, &humidity);
        if (res == ESP_OK)
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%", temperature, humidity);
        else
            ESP_LOGE(TAG, "Error reading data: %d (%s)", res, esp_err_to_name(res));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

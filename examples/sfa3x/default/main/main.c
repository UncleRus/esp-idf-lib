#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sfa3x.h>
#include <esp_log.h>
#include <esp_err.h>

static const char *TAG = "sfa3x-example";

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void task(void *pvParameters)
{
    i2c_dev_t dev = { 0 };

    ESP_ERROR_CHECK(sfa3x_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));

    char marking[32];
    ESP_ERROR_CHECK(sfa3x_get_device_marknig(&dev, marking));
    ESP_LOGI(TAG, "Sensor marking: %s", marking);

    ESP_ERROR_CHECK(sfa3x_start_continuous_measurement(&dev));
    ESP_LOGI(TAG, "Continuous measurement started");

    float hcho, temperature, humidity;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));

        esp_err_t res = sfa3x_read_measurement(&dev, &hcho, &humidity, &temperature);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading results %d (%s)", res, esp_err_to_name(res));
            continue;
        }

        ESP_LOGI(TAG, "Formaldehyde: %.2f bpm", hcho);
        ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
        ESP_LOGI(TAG, "Humidity: %.2f %%", humidity);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

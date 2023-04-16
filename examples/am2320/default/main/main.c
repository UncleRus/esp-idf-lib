#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <am2320.h>
#include <esp_log.h>
#include <inttypes.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "am2320-example";

void task(void *pvParameters)
{
    i2c_dev_t dev = {0};

    ESP_ERROR_CHECK(am2320_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    uint32_t dev_id;
    uint16_t model_id;
    uint8_t ver;
    ESP_ERROR_CHECK(am2320_get_device_id(&dev, &dev_id));
    ESP_ERROR_CHECK(am2320_get_model(&dev, &model_id));
    ESP_ERROR_CHECK(am2320_get_version(&dev, &ver));

    ESP_LOGI(TAG, "Found sensor! Device info:");
    ESP_LOGI(TAG, "Device ID: %" PRIX32, dev_id);
    ESP_LOGI(TAG, "Model ID:  %" PRIX16, model_id);
    ESP_LOGI(TAG, "Version:   %" PRIu8, ver);

    float temperature, humidity;

    while (1)
    {
        esp_err_t res = am2320_get_rht(&dev, &temperature, &humidity);
        if (res == ESP_OK)
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
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

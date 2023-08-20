#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <hdc1000.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "hdc1000-example";

void task(void *pvParameters)
{
    hdc1000_t dev = { 0 };

    ESP_ERROR_CHECK(hdc1000_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(hdc1000_init(&dev));

    uint32_t serial[2];
    uint16_t dev_id, manuf_id;

    ESP_ERROR_CHECK(hdc1000_get_serial(&dev, (uint64_t *)serial));
    ESP_ERROR_CHECK(hdc1000_get_manufacturer_id(&dev, &manuf_id));
    ESP_ERROR_CHECK(hdc1000_get_device_id(&dev, &dev_id));

    ESP_LOGI(TAG, "HDC1000, Manufacturer ID: 0x%04x, device ID: 0x%04x, serial number: 0x%08" PRIx32 "%08" PRIx32,
            manuf_id, dev_id, serial[0], serial[1]);

    float temperature, humidity;

    while (1)
    {
        esp_err_t res = hdc1000_measure(&dev, &temperature, &humidity);
        if (res == ESP_OK)
            ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", temperature, humidity);
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

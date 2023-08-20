#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <scd4x.h>
#include <esp_log.h>
#include <esp_err.h>

static const char *TAG = "scd4x-example";

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void task(void *pvParameters)
{
    i2c_dev_t dev = { 0 };

    ESP_ERROR_CHECK(scd4x_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    ESP_LOGI(TAG, "Initializing sensor...");
    ESP_ERROR_CHECK(scd4x_wake_up(&dev));
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&dev));
    ESP_ERROR_CHECK(scd4x_reinit(&dev));
    ESP_LOGI(TAG, "Sensor initialized");

    uint16_t serial[3];
    ESP_ERROR_CHECK(scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2));
    ESP_LOGI(TAG, "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    ESP_ERROR_CHECK(scd4x_start_periodic_measurement(&dev));
    ESP_LOGI(TAG, "Periodic measurements started");

    uint16_t co2;
    float temperature, humidity;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));

        esp_err_t res = scd4x_read_measurement(&dev, &co2, &temperature, &humidity);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading results %d (%s)", res, esp_err_to_name(res));
            continue;
        }

        if (co2 == 0)
        {
            ESP_LOGW(TAG, "Invalid sample detected, skipping");
            continue;
        }

        ESP_LOGI(TAG, "CO2: %u ppm", co2);
        ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
        ESP_LOGI(TAG, "Humidity: %.2f %%", humidity);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

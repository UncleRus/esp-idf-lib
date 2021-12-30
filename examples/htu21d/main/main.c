#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <htu21d.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 16
#define SCL_GPIO 17
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif
#define ADDR HTU21D_I2C_ADDRESS

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "htu21d-example";
void task(void *pvParameters)
{
    htu21d_t dev = {0};

    // Initialize HTU21D descriptor and initialize sensor
    ESP_ERROR_CHECK(htu21d_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(htu21d_init(&dev));

    float temperature, humidity;
    while (1)
    {
        esp_err_t res = htu21d_read_data(&dev, &temperature, &humidity);
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
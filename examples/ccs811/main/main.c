#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <ccs811.h>
#include <esp_log.h>
#include <esp_err.h>
#include <string.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "ccs811-example";

void test(void *pvParameters)
{
    ccs811_dev_t dev;
    memset(&dev, 0, sizeof(ccs811_dev_t)); // Zero descriptor

    ESP_LOGI(TAG, "Descriptor initialization");
    ESP_ERROR_CHECK(ccs811_init_desc(&dev, CCS811_I2C_ADDRESS_1, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Once I2C descreiptor initialized, function `ccs811_init()` has to be called
    // for each CCS811 sensor to initialize the sensor and to check its availability
    // as well as its error state.
    ESP_LOGI(TAG, "Sensor initialization");
    ESP_ERROR_CHECK(ccs811_init(&dev));

    // start periodic measurement with one measurement per second
    ESP_LOGI(TAG, "Setting measurement mode");
    ESP_ERROR_CHECK(ccs811_set_mode(&dev, CCS811_MODE_1S));

    esp_err_t res;
    while (1)
    {
        uint16_t tvoc, eco2;

        if ( (res = ccs811_get_results(&dev, &tvoc, &eco2, NULL, NULL)) == ESP_OK)
        {
            ESP_LOGI(TAG, "eCO2: %d ppm, TVOC: %d ppb", eco2, tvoc);
        }
        else
            ESP_LOGE(TAG, "Could not read resluts, error: %d - %s", res, esp_err_to_name(res));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("ccs811", ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

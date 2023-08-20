#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <tsl2561.h>
#include <esp_log.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "tsl2561-example";

void tsl2561_test(void *pvParameters)
{
    tsl2561_t dev = { 0 };

    ESP_ERROR_CHECK(tsl2561_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(tsl2561_init(&dev));

    ESP_LOGI(TAG, "Found TSL2561 in package %s", dev.package_type == TSL2561_PACKAGE_CS ? "CS" : "T/FN/CL");

    uint32_t lux;
    esp_err_t res;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        if ((res = tsl2561_read_lux(&dev, &lux)) != ESP_OK)
            ESP_LOGI(TAG, "Could not read illuminance value: %d (%s)", res, esp_err_to_name(res));
        else
            ESP_LOGI(TAG, "Illuminance: %" PRIu32 " Lux", lux);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(tsl2561_test, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

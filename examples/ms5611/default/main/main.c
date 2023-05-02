#include <inttypes.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ms5611.h>

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MS5611_ADDR_CSB_LOW
#endif
#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_HIGH
#define ADDR MS5611_ADDR_CSB_HIGH
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "ms5611-example";

void ms5611_test(void *pvParameters)
{
    ms5611_t dev = { 0 };

    ESP_ERROR_CHECK(ms5611_init_desc(&dev, ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(ms5611_init(&dev, MS5611_OSR_1024));

    float temperature;
    int32_t pressure;
    esp_err_t res;

    while (1)
    {
        // we can change oversampling ratio at any time:
        // dev.osr = MS5611_OSR_256

        vTaskDelay(pdMS_TO_TICKS(500));
        res = ms5611_get_sensor_data(&dev, &pressure, &temperature);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Temperature/pressure reading failed: %d (%s)", res, esp_err_to_name(res));
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        ESP_LOGI(TAG, "Pressure: %" PRIi32 " Pa, Temperature: %.2f C\n", pressure, temperature);
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(ms5611_test, "ms5611_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

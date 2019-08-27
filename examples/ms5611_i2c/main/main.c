#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ms5611.h>
#include <string.h>

#define I2C_ADDR MS5611_ADDR_CSB_LOW
#define I2C_PORT 0
#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif
#define OVERSAMPLING_RATIO MS5611_OSR_1024

void ms5611_test(void *pvParamters)
{
    ms5611_t dev;
    memset(&dev, 0, sizeof(ms5611_t));

    ESP_ERROR_CHECK(ms5611_init_desc(&dev, I2C_ADDR, I2C_PORT, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(ms5611_init(&dev, OVERSAMPLING_RATIO));

    float temperature;
    int32_t pressure;

    while (1)
    {
        // we can change oversampling ratio at any time:
        // dev.osr = MS5611_OSR_256

        vTaskDelay(500 / portTICK_PERIOD_MS);
        if (ms5611_get_sensor_data(&dev, &pressure, &temperature) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("Pressure: %d Pa, Temperature: %.2f C\n", pressure, temperature);
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(ms5611_test, "ms5611_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


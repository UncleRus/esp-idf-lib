#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ms5611.h>

#define I2C_ADDR MS5611_ADDR_CSB_LOW
#define I2C_PORT 0
#define SDA_GPIO 16
#define SCL_GPIO 17
#define OVERSAMPLING_RATIO MS5611_OSR_1024

void ms5611_test(void *pvParamters)
{
    ms5611_t dev;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (ms5611_init_desc(&dev, I2C_ADDR, I2C_PORT, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    esp_err_t res;
    while ((res = ms5611_init(&dev, OVERSAMPLING_RATIO)) != ESP_OK)
    {
        printf("Could not init MS5611-01BA03, err: %d\n", res);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

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

        printf("Pressure: %d Pa, Temperature: %.2f C\n", pressure, temperature);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(ms5611_test, "ms5611_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <max31725.h>
#include <string.h>

#define CONTINUOUS 0

#define I2C_PORT 0

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#define ADDRESS MAX31725_I2C_ADDR_BASE
#define FORMAT MAX31725_FMT_NORMAL

// Main task
void test(void *pvParamters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(max31725_init_desc(&dev, I2C_PORT, ADDRESS, SDA_GPIO, SCL_GPIO));

#if CONTINUOUS

    // Continuous measurement mode

    ESP_ERROR_CHECK(max31725_set_config(&dev, MAX31725_MODE_CONTINUOUS, FORMAT,
            MAX31725_FAULTS_1, MAX31725_OS_LOW, MAX31725_OS_COMPARATOR));
    while (1)
    {
        float temp;
        if (max31725_get_temperature(&dev, &temp, FORMAT) == ESP_OK)
            printf("Temperature: %.02f deg.C\n", temp);
        else
            printf("Could not read temperature\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
#else

    // One-shot measurement mode

    ESP_ERROR_CHECK(max31725_set_config(&dev, MAX31725_MODE_SHUTDOWN, FORMAT,
            MAX31725_FAULTS_1, MAX31725_OS_LOW, MAX31725_OS_COMPARATOR));
    while (1)
    {
        float temp;
        if (max31725_one_shot(&dev, &temp, FORMAT) == ESP_OK)
            /* float is used in printf(). you need non-default configuration in
             * sdkconfig for ESP8266, which is enabled by default for this
             * example. see sdkconfig.defaults.esp8266
             */
            printf("Temperature: %.02f deg.C\n", temp);
        else
            printf("Could not read temperature\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif
}

void app_main()
{
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());
    // Start task
    xTaskCreatePinnedToCore(test, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


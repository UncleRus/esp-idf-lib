#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <si7021.h>
#include <string.h>

#define CHIP_TYPE_SI70xx  // comment this line for SHT2x/HTU21D

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void task(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(si7021_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));

#ifdef CHIP_TYPE_SI70xx
    uint64_t serial;
    si7021_device_id_t id;

    ESP_ERROR_CHECK(si7021_get_serial(&dev, &serial, false));
    ESP_ERROR_CHECK(si7021_get_device_id(&dev, &id));

    printf("Device: ");
    switch (id)
    {
        case SI_MODEL_SI7013:
            printf("Si7013");
            break;
        case SI_MODEL_SI7020:
            printf("Si7020");
            break;
        case SI_MODEL_SI7021:
            printf("Si7021");
            break;
        case SI_MODEL_SAMPLE:
            printf("Engineering sample");
            break;
        default:
            printf("Unknown");
    }
    printf("\nSerial number: 0x%08x%08x\n", (uint32_t)(serial >> 32), (uint32_t)serial);
#endif

    float val;
    esp_err_t res;

    while (1)
    {
        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        res = si7021_measure_temperature(&dev, &val);
        if (res != ESP_OK)
            printf("Could not measure temperature: %d (%s)\n", res, esp_err_to_name(res));
        else
            printf("Temperature: %.2f\n", val);

        res = si7021_measure_humidity(&dev, &val);
        if (res != ESP_OK)
            printf("Could not measure humidity: %d (%s)\n", res, esp_err_to_name(res));
        else
            printf("Humidity: %.2f\n", val);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

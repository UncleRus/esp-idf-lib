#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pcf8591.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif
#define ADDR PCF8591_DEFAULT_ADDRESS

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void pcf8591_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(pcf8591_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO));

    while (1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        esp_err_t res;
        uint8_t val;
        if ((res = pcf8591_read(&dev, PCF8591_IC_4_SINGLES, 3, &val)) != ESP_OK)
            printf("Could not read ADC value, error %d\n", res);
        else
            printf("Value: %d\n", val);
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(pcf8591_test, "pcf8591_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


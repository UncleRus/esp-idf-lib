#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pcf8591.h>

#define SDA_GPIO 16
#define SCL_GPIO 17
#define ADDR PCF8591_DEFAULT_ADDRESS

void pcf8591_test(void *pvParamters)
{
    i2c_dev_t dev;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (pcf8591_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

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
    xTaskCreatePinnedToCore(pcf8591_test, "pcf8591_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


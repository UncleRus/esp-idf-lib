#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <bh1900nux.h>
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

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define ADDRESS BH1900NUX_I2C_ADDR_BASE

// Main task
void test(void *pvParameters)
{
    bh1900nux_t dev;
    memset(&dev, 0, sizeof(bh1900nux_t));

    ESP_ERROR_CHECK(bh1900nux_init_desc(&dev, ADDRESS, I2C_PORT, SDA_GPIO, SCL_GPIO));

#if CONTINUOUS

    // Continuous measurement mode
    ESP_ERROR_CHECK(bh1900nux_set_mode(&dev, BH1900NUX_MODE_CONTINUOUS));

    // delay between measurements = 414ms
    // single fault data queue
    // ALERT pin active polarity = low
    ESP_ERROR_CHECK(bh1900nux_set_config(&dev, BH1900NUX_WT_2, BH1900NUX_FAULTS_1, BH1900NUX_ALERT_LOW));
    while (1)
    {
        float temp;
        if (bh1900nux_get_temperature(&dev, &temp) == ESP_OK)
            printf("Temperature: %.02f deg.C\n", temp);
        else
            printf("Could not read temperature\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
#else

    // One-shot measurement mode
    ESP_ERROR_CHECK(bh1900nux_set_mode(&dev, BH1900NUX_MODE_SHUTDOWN));

    // delay between measurements = does not matter
    // single fault data queue
    // ALERT pin active polarity = low
    ESP_ERROR_CHECK(bh1900nux_set_config(&dev, BH1900NUX_WT_0, BH1900NUX_FAULTS_1, BH1900NUX_ALERT_LOW));
    while (1)
    {
        float temp;
        esp_err_t r = bh1900nux_one_shot(&dev, &temp);
        if (r == ESP_OK)
            /* float is used in printf(). you need non-default configuration in
             * sdkconfig for ESP8266, which is enabled by default for this
             * example. see sdkconfig.defaults.esp8266
             */
            printf("Temperature: %.02f deg.C\n", temp);
        else
            printf("Could not read temperature: %d (%s)\n", r, esp_err_to_name(r));

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


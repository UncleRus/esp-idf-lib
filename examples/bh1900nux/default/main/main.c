#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <bh1900nux.h>
#include <string.h>

#define I2C_PORT 0

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

// Main task
void test(void *pvParameters)
{
    bh1900nux_t dev;
    memset(&dev, 0, sizeof(bh1900nux_t));

    ESP_ERROR_CHECK(bh1900nux_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

#if CONFIG_EXAMPLE_MEASURING_MODE_CONTINUOUS

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


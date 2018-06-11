#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ads111x.h>

#define SDA_GPIO 16
#define SCL_GPIO 17
#define ADDRESS ADS111X_ADDR_GND // connect ADDR pin to GND
#define GAIN ADS111X_GAIN_4V096

void ads111x_test(void *pvParamters)
{
    i2c_dev_t dev;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (ads111x_init_desc(&dev, ADDRESS, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    ads111x_set_mode(&dev, ADS111X_MODE_CONTUNOUS);
    ads111x_set_data_rate(&dev, ADS111X_DATA_RATE_32);

    ads111x_set_input_mux(&dev, ADS111X_MUX_0_GND);
    ads111x_set_gain(&dev, GAIN);

    float gain_val = ads111x_gain_values[GAIN];

    while (1)
    {
        // wait for conversion end
        bool busy;
        do
        {
            ads111x_is_busy(&dev, &busy);
        }
        while (busy);

        // Read result
        int16_t raw = 0;
        if (ads111x_get_value(&dev, &raw) == ESP_OK)
        {
            float voltage = gain_val / ADS111X_MAX_VALUE * raw;
            printf("Raw ADC value: %d, voltage: %.04f volts\n", raw, voltage);
        }
        else
            printf("Cannot read ADC value\n");


        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(ads111x_test, "ads111x_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


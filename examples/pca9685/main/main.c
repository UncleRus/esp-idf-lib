#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <pca9685.h>

#define ADDR PCA9685_ADDR_BASE
#define SDA_GPIO 16
#define SCL_GPIO 17

void pca9685_test(void *pvParamters)
{
    i2c_dev_t dev;
    esp_err_t res;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (pca9685_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while ((res = pca9685_init(&dev)) != ESP_OK)
    {
        printf("Could not init PCA9685, err: %d\n", res);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    pca9685_set_pwm_frequency(&dev, 1000);
    uint16_t freq;
    pca9685_get_pwm_frequency(&dev, &freq);
    printf("Freq 1000Hz, real %d\n", freq);

    uint16_t val = 0;
    while (1)
    {
        printf("Set ch0 to %d, ch4 to %d\n", val, 4096 - val);
        pca9685_set_pwm_value(&dev, 0, val);
        pca9685_set_pwm_value(&dev, 4, 4096 - val);

        if (val++ == 4096)
            val = 0;
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(pca9685_test, "pca9685_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


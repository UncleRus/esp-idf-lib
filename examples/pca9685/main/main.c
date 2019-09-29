#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pca9685.h>
#include <string.h>

#define ADDR PCA9685_ADDR_BASE
#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

void pca9685_test(void *pvParamters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(pca9685_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&dev));

    ESP_ERROR_CHECK(pca9685_restart(&dev));

    uint16_t freq;
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, 1000));
    ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&dev, &freq));
    printf("Freq 1000Hz, real %d\n", freq);

    uint16_t val = 0;
    while (1)
    {
        printf("Set ch0 to %d, ch4 to %d\n", val, 4096 - val);

        if (pca9685_set_pwm_value(&dev, 0, val) != ESP_OK)
            printf("Could not set PWM value to ch0\n");
        if (pca9685_set_pwm_value(&dev, 4, 4096 - val) != ESP_OK)
            printf("Could not set PWM value to ch4");

        if (val++ == 4096)
            val = 0;
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(pca9685_test, "pca9685_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


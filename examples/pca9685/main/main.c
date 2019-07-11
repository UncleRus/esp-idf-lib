#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pca9685.h>
#include <string.h>

#define ADDR PCA9685_ADDR_BASE
#define SDA_GPIO 16
#define SCL_GPIO 17

#define CHECK_LOOP(X, msg, ...) do { esp_err_t __; while((__ = X) != ESP_OK) { printf(msg "\n", ## __VA_ARGS__); vTaskDelay(250 / portTICK_PERIOD_MS); }} while (0)

void pca9685_test(void *pvParamters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    CHECK_LOOP(i2cdev_init(),
            "Could not init I2Cdev library");
    CHECK_LOOP(pca9685_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO),
            "Could not init device descriptor");
    CHECK_LOOP(pca9685_init(&dev),
            "Could not init PCA9685");
    CHECK_LOOP(pca9685_restart(&dev),
            "Could not restart");

    CHECK_LOOP(pca9685_set_pwm_frequency(&dev, 1000),
            "Could not set PWM frequency");
    uint16_t freq;
    CHECK_LOOP(pca9685_get_pwm_frequency(&dev, &freq),
            "Could not get PWM frequency");
    printf("Freq 1000Hz, real %d\n", freq);

    uint16_t val = 0;
    while (1)
    {
        printf("Set ch0 to %d, ch4 to %d\n", val, 4096 - val);
        CHECK_LOOP(pca9685_set_pwm_value(&dev, 0, val),
                "Could not set PWM value to ch0");
        CHECK_LOOP(pca9685_set_pwm_value(&dev, 4, 4096 - val),
                "Could not set PWM value to ch4");

        if (val++ == 4096)
            val = 0;
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(pca9685_test, "pca9685_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


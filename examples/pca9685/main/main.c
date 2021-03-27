#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pca9685.h>
#include <string.h>
#include <esp_log.h>

#define ADDR PCA9685_ADDR_BASE
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

#define PWM_FREQ_HZ 1500

static const char *TAG = "pca9685_test";

void pca9685_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(pca9685_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&dev));

    ESP_ERROR_CHECK(pca9685_restart(&dev));

    uint16_t freq;
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, PWM_FREQ_HZ));
    ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&dev, &freq));

    ESP_LOGI(TAG, "Freq %dHz, real %d", PWM_FREQ_HZ, freq);

    uint16_t val = 0;
    while (1)
    {
        if (!(val % 100))
            ESP_LOGI(TAG, "CH0 = %-4d | CH3 = %-4d", val, PCA9685_MAX_PWM_VALUE - val);

        if (pca9685_set_pwm_value(&dev, 0, val) != ESP_OK)
            ESP_LOGE(TAG, "Could not set PWM value to ch0");
        if (pca9685_set_pwm_value(&dev, 4, PCA9685_MAX_PWM_VALUE - val) != ESP_OK)
            ESP_LOGE(TAG, "Could not set PWM value to ch3");

        if (val++ > PCA9685_MAX_PWM_VALUE - 1)
            val = 0;
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(pca9685_test, TAG, configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);
}


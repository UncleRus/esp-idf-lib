#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pca9557.h>
#include <driver/gpio.h>
#include <string.h>

// A0, A1, A2 pins are grounded
#define ADDR PCA9557_I2C_ADDR_BASE

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define INT_GPIO 16
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define INT_GPIO 19
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

void test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(pca9557_init_desc(&dev, 0, PCA9557_I2C_ADDR_BASE, SDA_GPIO, SCL_GPIO));

    // Setup IO0-IO3 as inputs, IO4-IO7 as outputs
    ESP_ERROR_CHECK(pca9557_port_set_mode(&dev, 0x0f));

    // blink on IO7 and read IO0
    bool on = true;
    while (1)
    {
        ESP_ERROR_CHECK(pca9557_set_level(&dev, 7, on));
        on = !on;
        uint8_t in = 0;
        ESP_ERROR_CHECK(pca9557_port_read(&dev, &in));
        printf("IO0: %s\n", (in & 1) ? "high" : "low");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}


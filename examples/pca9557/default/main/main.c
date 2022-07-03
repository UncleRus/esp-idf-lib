#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pca9557.h>
#include <driver/gpio.h>

void test(void *pvParameters)
{
    i2c_dev_t dev = { 0 };

    ESP_ERROR_CHECK(pca9557_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Setup IO0 as input, IO3 as output
    // instead of calling pca9557_set_mode() multiple times, you can call pca9557_port_set_mode()
    ESP_ERROR_CHECK(pca9557_set_mode(&dev, 0, PCA9557_MODE_INPUT));
    ESP_ERROR_CHECK(pca9557_set_mode(&dev, 3, PCA9557_MODE_OUTPUT));

    // blink on IO3 and read IO0
    bool on = true;
    while (1)
    {
        ESP_ERROR_CHECK(pca9557_set_level(&dev, 3, on));
        on = !on;
        uint8_t in = 0;
        ESP_ERROR_CHECK(pca9557_port_read(&dev, &in)); // you can use pca9557_get_level() instead
        printf("IO0: %s\n", (in & 1) ? "high" : "low");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}


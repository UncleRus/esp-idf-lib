#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pcf8575.h>
#include <string.h>

#define I2C_ADDR PCF8575_I2C_ADDR_BASE

void test(void *pvParameters)
{
    i2c_dev_t pcf8575;

    // Zero device descriptor
    memset(&pcf8575, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8575_init_desc(&pcf8575, I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Do some blinking
    uint16_t port_val = 0xaaaa;
    while (1)
    {
        // invert value
        port_val = ~port_val;

        // write value to port
        pcf8575_port_write(&pcf8575, port_val);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}


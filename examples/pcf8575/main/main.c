#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pcf8575.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#define I2C_ADDR PCF8575_I2C_ADDR_BASE

void test(void *pvParameters)
{
    i2c_dev_t pcf8575;

    // Zero device descriptor
    memset(&pcf8575, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8575_init_desc(&pcf8575, 0, I2C_ADDR, SDA_GPIO, SCL_GPIO));

    // Do some blinking
    uint16_t port_val = 0xaaaa;
    while (1)
    {
        // invert value
        port_val = ~port_val;

        // write value to port
        pcf8575_port_write(&pcf8575, port_val);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}


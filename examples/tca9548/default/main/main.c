#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <tca9548.h>
#include <bmp180.h>
#include <string.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

// We have two BMP180s connected to the channels 0 and 1 of TCA9548
#define SENSOR_COUNT 2
static bmp180_dev_t sensors[SENSOR_COUNT] = { 0 };
static i2c_dev_t i2c_switch = { 0 };

void i2c_switch_test(void *pvParameters)
{
    // Initialize sensor descriptors
    for (size_t i = 0; i < SENSOR_COUNT; i++)
        ESP_ERROR_CHECK(bmp180_init_desc(&sensors[i], 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Initialize descriptor of the I2C switch
    ESP_ERROR_CHECK(tca9548_init_desc(&i2c_switch, CONFIG_EXAMPLE_SWITCH_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Initialize sensors
    for (size_t i = 0; i < SENSOR_COUNT; i++)
    {
        ESP_ERROR_CHECK(tca9548_set_channels(&i2c_switch, BIT(i)));
        ESP_ERROR_CHECK(bmp180_init(&sensors[i]));
    }

    // Measure loop
    while (1)
    {
        float temp;
        uint32_t pressure;

        for (size_t i = 0; i < SENSOR_COUNT; i++)
        {
            ESP_ERROR_CHECK(tca9548_set_channels(&i2c_switch, BIT(i)));
            esp_err_t res = bmp180_measure(&sensors[i], &temp, &pressure, BMP180_MODE_STANDARD);
            if (res != ESP_OK)
                printf("Could not measure on sensor %d: %d\n", i, res);
            else
                /* float is used in printf(). you need non-default configuration in
                 * sdkconfig for ESP8266, which is enabled by default for this
                 * example. see sdkconfig.defaults.esp8266
                 */
                printf("[Sensor %d] Temperature: %.2f degrees Celsius; Pressure: %" PRIu32 " Pa\n", i, temp, pressure);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(i2c_switch_test, "i2c_switch_test", configMINIMAL_STACK_SIZE * 15, NULL, 5, NULL, APP_CPU_NUM);
}

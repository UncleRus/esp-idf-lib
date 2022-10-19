#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <icm42670.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define I2C_MASTER_SDA 10
#define I2C_MASTER_SCL 8

void icm42670_test(void *pvParameters)
{
    icm42670_t dev = { 0 };

    ESP_ERROR_CHECK(icm42670_init_desc(&dev, ICM42670_I2C_ADDR_GND, 0, I2C_MASTER_SDA, I2C_MASTER_SCL)); // TODO add CONFIG_EXAMPLE_ prefix for pins
    ESP_ERROR_CHECK(icm42670_init(&dev));

    float temperature;
    esp_err_t res;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        if ((res = icm42670_read_temperature(&dev, &temperature)) != ESP_OK)
            printf("Could not read temperature value: %d\n", res);
        else
            printf("Temperature: %f\n", temperature);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(icm42670_test, "icm42670_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


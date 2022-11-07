#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "driver/gpio.h"
#include <icm42670.h>

//#include "esp_sleep.h"
//#include "esp_log.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define I2C_MASTER_SDA 10
#define I2C_MASTER_SCL 8

#define INT_INPUT_PIN 0

void icm42670_test(void *pvParameters)
{
    // config IO0 as input, pull-up enabled
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT(INT_INPUT_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    icm42670_t dev = { 0 };
    // init device descriptor and device
    ESP_ERROR_CHECK(icm42670_init_desc(&dev, ICM42670_I2C_ADDR_GND, 0, I2C_MASTER_SDA, I2C_MASTER_SCL)); // TODO add CONFIG_EXAMPLE_ prefix for pins
    ESP_ERROR_CHECK(icm42670_init(&dev));

    // config a WoM interrupt on pin 2
    const icm42670_int_config_t int_config = {
        .mode = ICM42670_INT_MODE_LATCHED,
        .drive = ICM42670_INT_DRIVE_PUSH_PULL,
        .polarity = ICM42670_INT_POLARITY_ACTIVE_LOW,
    };
    const uint8_t int_pin = 2;

    ESP_ERROR_CHECK(icm42670_config_int_pin(&dev, int_pin, int_config));

    icm42670_int_source_t sources = {false};
    sources.wom_z = true;
    sources.wom_y = true;
    sources.wom_z = true;

    ESP_ERROR_CHECK(icm42670_set_int_sources(&dev, int_pin, sources));

    const icm42670_wom_config_t wom_config = {
        .trigger = ICM42670_WOM_INT_DUR_FIRST,
        .logical_mode = ICM42670_WOM_INT_MODE_ALL_OR,
        .reference = ICM42670_WOM_MODE_REF_LAST,
        .wom_x_threshold = 30,
        .wom_y_threshold = 30,
        .wom_z_threshold = 30,
    };

    ESP_ERROR_CHECK(icm42670_config_wom(&dev, wom_config));

    // enable averaging on accel
    ESP_ERROR_CHECK(icm42670_set_accel_avg(&dev, ICM42670_ACCEL_AVG_8X));
    // disable gyro and enable accel in low-power mode
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_DISABLE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LP_MODE));

    ESP_ERROR_CHECK(icm42670_enable_wom(&dev, true));

    while(1)
    {
        printf("GPIO level pin 0: %d\n", gpio_get_level(INT_INPUT_PIN));
        vTaskDelay(pdMS_TO_TICKS(100));
    }



    /* TEMPERATURE EXAMPLE
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
    */
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(icm42670_test, "icm42670_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


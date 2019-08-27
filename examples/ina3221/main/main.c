#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ina3221.h>
#include <string.h>

#define I2C_PORT 0
#define I2C_ADDR INA3221_I2C_ADDR_GND
#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#define WARNING_CHANNEL 1
#define WARNING_CURRENT (40.0)

//#define STRUCT_SETTING 0
#define MODE false  // true : continuous  measurements // false : trigger measurements

void task(void *pvParameters)
{
    ina3221_t dev = {
            .shunt = { 100, 100, 100 },         // shunt values are 100 mOhm for each channel
            .config.config_register = INA3221_DEFAULT_CONFIG,
            .mask.mask_register = INA3221_DEFAULT_MASK
    };
    memset(&dev.i2c_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(ina3221_init_desc(&dev, I2C_ADDR, I2C_PORT, SDA_GPIO, SCL_GPIO));

#ifndef STRUCT_SETTING
    ESP_ERROR_CHECK(ina3221_set_options(&dev, MODE, true, true));    // Mode selection, bus and shunt activated
    ESP_ERROR_CHECK(ina3221_enable_channel(&dev, true, true, true)); // Enable all channels
    ESP_ERROR_CHECK(ina3221_set_average(&dev, INA3221_AVG_64));      // 64 samples average
    ESP_ERROR_CHECK(ina3221_set_bus_conversion_time(&dev, INA3221_CT_2116));   // 2ms by channel
    ESP_ERROR_CHECK(ina3221_set_shunt_conversion_time(&dev, INA3221_CT_2116)); // 2ms by channel
#else
    dev.config.mode = MODE; // mode selection
    dev.config.esht = true; // shunt enable
    dev.config.ebus = true; // bus enable
    dev.config.ch1 = true;  // channel 1 enable
    dev.config.ch2 = true;  // channel 2 enable
    dev.config.ch3 = true;  // channel 3 enable
    dev.config.avg = INA3221_AVG_64;   // 64 samples average
    dev.config.vbus = INA3221_CT_2116; // 2ms by channel (bus)
    dev.config.vsht = INA3221_CT_2116; // 2ms by channel (shunt)
    ESP_ERROR_CHECK(ina3221_sync(&dev));
#endif

    ESP_ERROR_CHECK(ina3221_set_warning_alert(&dev, WARNING_CHANNEL - 1, WARNING_CURRENT)); // Set overcurrent security flag

    uint32_t measure_number = 0;
    bool warning = false;
    float bus_voltage;
    float shunt_voltage;
    float shunt_current;

    while (1)
    {
        measure_number++;

#if !MODE
        ESP_ERROR_CHECK(ina3221_trigger(&dev)); // Start a measure
        printf("trig done, wait: ");
        do
        {
            printf("X");

            ESP_ERROR_CHECK(ina3221_get_status(&dev)); // get mask

            if (dev.mask.wf & (1 << (3 - WARNING_CHANNEL)))
                warning = true;

            vTaskDelay(20 / portTICK_PERIOD_MS);

        } while (!(dev.mask.cvrf)); // check if measure done
#else
        ESP_ERROR_CHECK(ina3221_get_status(&dev)); // get mask

        if (dev.mask.wf & (1 << (3 - WARNING_CHANNEL)))
            warning = true;
#endif
        for (uint8_t i = 0; i < INA3221_BUS_NUMBER; i++)
        {
            // Get voltage in volts
            ESP_ERROR_CHECK(ina3221_get_bus_voltage(&dev, i, &bus_voltage));
            // Get voltage in millivolts and current in milliamperes
            ESP_ERROR_CHECK(ina3221_get_shunt_value(&dev, i, &shunt_voltage, &shunt_current));

            printf("\nC%u:Measure number %u\n", i + 1, measure_number);
            if (warning && (i + 1) == WARNING_CHANNEL)
                printf("C%u:Warning Current > %.2f mA !!\n", i + 1, WARNING_CURRENT);
            printf("C%u:Bus voltage: %.02f V\n", i + 1, bus_voltage);
            printf("C%u:Shunt voltage: %.02f mV\n", i + 1, shunt_voltage);
            printf("C%u:Shunt current: %.02f mA\n\n", i + 1, shunt_current);

        }
        warning = false ;

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(task, "ina3221_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}


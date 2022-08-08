#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ina3221.h>
#include <string.h>

#define I2C_PORT 0
#define WARNING_CHANNEL 1
#define WARNING_CURRENT (40.0)

//#define STRUCT_SETTING 0
#if defined EXAMPLE_MEASURING_MODE_TRIGGER
#define MODE false  // true : continuous  measurements // false : trigger measurements
#else
#define MODE true
#endif

void task(void *pvParameters)
{
    ina3221_t dev = {
            /* shunt values are 100 mOhm for each channel */
            .shunt = {
                CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM,
                CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM,
                CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM
            },
            .config.config_register = INA3221_DEFAULT_CONFIG,
            .mask.mask_register = INA3221_DEFAULT_MASK
    };
    memset(&dev.i2c_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(ina3221_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

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

#if CONFIG_EXAMPLE_MEASURING_MODE_TRIGGER
        ESP_ERROR_CHECK(ina3221_trigger(&dev)); // Start a measure
        printf("trig done, wait: ");
        do
        {
            printf("X");

            ESP_ERROR_CHECK(ina3221_get_status(&dev)); // get mask

            if (dev.mask.wf & (1 << (3 - WARNING_CHANNEL)))
                warning = true;

            vTaskDelay(pdMS_TO_TICKS(20));

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

            printf("\nC%u:Measure number %" PRIu32 "\n", i + 1, measure_number);
            if (warning && (i + 1) == WARNING_CHANNEL)
                printf("C%u:Warning Current > %.2f mA !!\n", i + 1, WARNING_CURRENT);
            printf("C%u:Bus voltage: %.02f V\n", i + 1, bus_voltage);
            printf("C%u:Shunt voltage: %.02f mV\n", i + 1, shunt_voltage);
            printf("C%u:Shunt current: %.02f mA\n\n", i + 1, shunt_current);

        }
        warning = false ;

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    xTaskCreate(task, "ina3221_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

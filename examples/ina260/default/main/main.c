#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ina260.h>
#include <string.h>
#include <esp_log.h>

#define I2C_PORT 0
#define I2C_ADDR CONFIG_EXAMPLE_I2C_ADDR

void wait_for_sensor(ina260_t *dev)
{
    bool ready, alert, overflow;
    while (1)
    {
        ESP_ERROR_CHECK(ina260_get_status(dev, &ready, &alert, &overflow));
        if (alert) printf("ALERT! ");
        if (overflow) printf("OVERFLOW! ");
        if (alert || overflow) printf("\n");
        if (ready) break;
    }
}

void task(void *pvParameters)
{
    ina260_t dev;
    memset(&dev, 0, sizeof(ina260_t));

    ESP_ERROR_CHECK(ina260_init_desc(&dev, I2C_ADDR, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    printf("Initializing ina260\n");
    ESP_ERROR_CHECK(ina260_init(&dev));

    printf("Configuring ina260\n");
    // Triggered mode, 128 samples, 1.1 ms conversion time (~150 ms)
    ESP_ERROR_CHECK(ina260_set_config(&dev, INA260_MODE_TRIG_SHUNT_BUS, INA260_AVG_128,
            INA260_CT_1100, INA260_CT_1100));

    wait_for_sensor(&dev);

    float voltage, current, power;
    while (1)
    {
        ESP_ERROR_CHECK(ina260_trigger(&dev));

        wait_for_sensor(&dev);

        ESP_ERROR_CHECK(ina260_get_bus_voltage(&dev, &voltage));
        ESP_ERROR_CHECK(ina260_get_current(&dev, &current));
        ESP_ERROR_CHECK(ina260_get_power(&dev, &power));
        /* Using float in printf() requires non-default configuration in
         * sdkconfig. See sdkconfig.defaults.esp32 and
         * sdkconfig.defaults.esp8266  */
        printf("VBUS: %.04f V, IBUS: %.04f mA, PBUS: %.04f mW\n",
                voltage, current * 1000, power * 1000);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}


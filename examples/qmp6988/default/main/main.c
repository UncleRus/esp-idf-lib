/**
 * Simple example with QMP6988 sensor.
 *
 * It shows different user task implementations in *forced mode* and
 * *normal mode*.
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <qmp6988.h>
#include <string.h>
#include <esp_err.h>

/* float is used in printf(). you need non-default configuration in
 * sdkconfig for ESP8266, which is enabled by default for this
 * example. see sdkconfig.defaults.esp8266
 */

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static qmp6988_t dev;

#if defined(CONFIG_EXAMPLE_QMP6988_DEMO_FORCED)

/*
 * User task that triggers a measurement every 5 seconds. Due to power
 * efficiency reasons it uses *Forced* mode.
 */
void task(void *pvParameters)
{
    float temperature;
    float pressure;

    TickType_t last_wakeup = xTaskGetTickCount();

    /* We can lower the standard filter and oversampling to use less power
    and in low accuracy examples e.g. a weather station */

    ESP_ERROR_CHECK(qmp6988_set_filter(&dev, QMP6988_FILTERCOEFF_OFF));
    ESP_ERROR_CHECK(qmp6988_set_p_oversampling(&dev, QMP6988_OVERSAMPLING_2X));
    ESP_ERROR_CHECK(qmp6988_set_t_oversampling(&dev, QMP6988_OVERSAMPLING_1X));

    while (1)
    {
        /*Set forced mode for qmp6988 (need to set this each time as it falls
        back to sleep mode automatically after a single measurement). */
        if (qmp6988_setup_powermode(&dev, QMP6988_FORCED_MODE) == ESP_OK)
            printf("Power mode set to forced mode\n");
        else
            printf("Sensor error\n");

        /* Wait until forced measurement is ready (constant time of at least 5.5 ms
        according to the modified oversampling and filter modes.*/
        vTaskDelay(1);

        /*Calculate pressure values (this includes temperature as well,
        as temp is needed to calc pressure)*/
        pressure = qmp6988_calc_pressure(&dev);
        temperature = dev.temperature;
        printf("QMP6988 Sensor: %.2f °C, %.2f Pa\n", temperature, pressure);

        // Wait until 5 seconds (cycle time) are over.
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(5000));
    }
}

#else // CONFIG_QMP6988_DEMO_NORMAL
/*
 * User task that fetches latest measurement results of sensor every 2
 * seconds. It starts the QMP6988 in normal (periodic) mode.
 */
void task(void *pvParameters)
{
    float temperature;
    float pressure;

    // Set normal mode for qmp6988.
    if (qmp6988_setup_powermode(&dev, QMP6988_NORMAL_MODE) == ESP_OK)
        printf("Power mode set to normal mode\n");
    else
        printf("Sensor error\n");

    /* Wait until first measurement is ready (constant time of at least 10.6 ms
    according to the default oversampling and filter modes.*/
    vTaskDelay(2);

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        /*Calculate pressure values (this includes temperature as well,
        as temp is needed to calc pressure)*/
        pressure = qmp6988_calc_pressure(&dev);
        temperature = dev.temperature;
        printf("QMP6988 Sensor: %.2f °C, %.2f Pa\n", temperature, pressure);

        // Wait until 2 seconds (cycle time) are over.
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(2000));
    }
}

#endif

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    ESP_ERROR_CHECK(qmp6988_init_desc(&dev, CONFIG_EXAMPLE_QMP6988_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(qmp6988_init(&dev));

    xTaskCreatePinnedToCore(task, "qmp6988_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

/**
 * Simple example with SHT4x sensor.
 *
 * It shows different user task implementation: either low level or high level
 * functions are used.
 *
 * Hardware configuration:
 *
 *    +-----------------+     +----------+
 *    |     ESP32       |     | SHT4x    |
 *    |                 |     |          |
 *    |   GPIO 16 (SCL) ------> SCL      |
 *    |   GPIO 17 (SDA) <-----> SDA      |
 *    +-----------------+     +----------+
 */
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <sht4x.h>
#include <string.h>
#include <esp_err.h>

/* float is used in printf(). you need non-default configuration in
 * sdkconfig for ESP8266, which is enabled by default for this
 * example. see sdkconfig.defaults.esp8266
 */

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static sht4x_t dev;

#if defined(CONFIG_SHT4X_DEMO_LL)

/*
 * User task that triggers a measurement every 5 seconds.
 * In this example it starts the measurement, waits for the results and
 * fetches the results using separate functions
 */
void task(void *pvParameters)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    // get the measurement duration for high repeatability;
    uint8_t duration = sht4x_get_measurement_duration(dev);

    while (1)
    {
        // Trigger one measurement in single shot mode with high repeatability.
        ESP_ERROR_CHECK(sht4x_start_measurement(&dev));

        // Wait until measurement is ready (duration returned from *sht4x_get_measurement_duration*).
        vTaskDelay(duration);

        // retrieve the values and do something with them
        ESP_ERROR_CHECK(sht4x_get_results(&dev, &temperature, &humidity));
        printf("sht4x Sensor: %.2f °C, %.2f %%\n", temperature, humidity);

        // wait until 5 seconds are over
        vTaskDelayUntil(&last_wakeup, 5000 / portTICK_PERIOD_MS);
    }
}

#else
/*
 * User task that triggers a measurement every 5 seconds.
 * In this example it uses the high level function *sht4x_measure*
 * to perform one measurement in each cycle.
 */
void task(void *pvParameters)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        // perform one measurement and do something with the results
        ESP_ERROR_CHECK(sht4x_measure(&dev, &temperature, &humidity));
        printf("sht4x Sensor: %.2f °C, %.2f %%\n", temperature, humidity);

        // wait until 5 seconds are over
        vTaskDelayUntil(&last_wakeup, 5000 / portTICK_PERIOD_MS);
    }
}

#endif

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&dev, 0, sizeof(sht4x_t));

    ESP_ERROR_CHECK(sht4x_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(sht4x_init(&dev));

    xTaskCreatePinnedToCore(task, "sht4x_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

/**
 * Simple example with SHT3x sensor.
 *
 * It shows different user task implementations in *single shot mode* and
 * *periodic mode*. In *single shot* mode either low level or high level
 * functions are used.
 *
 * Harware configuration:
 *
 *    +-----------------+     +----------+
 *    |     ESP32 |     | SHT3x    |
 *    |                 |     |          |
 *    |   GPIO 16 (SCL) ------> SCL      |
 *    |   GPIO 17 (SDA) <-----> SDA      |
 *    +-----------------+     +----------+
 */
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <sht3x.h>
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

#define ADDR SHT3X_I2C_ADDR_GND

static sht3x_t dev;

#if defined(CONFIG_SHT3X_DEMO_SSHL)

/*
 * User task that triggers a measurement every 5 seconds. Due to power
 * efficiency reasons it uses *single shot* mode. In this example it uses the
 * high level function *sht3x_measure* to perform one measurement in each cycle.
 */
void task(void *pvParamters)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        // perform one measurement and do something with the results
        ESP_ERROR_CHECK(sht3x_measure(&dev, &temperature, &humidity));
        printf("SHT3x Sensor: %.2f °C, %.2f %%\n", temperature, humidity);

        // wait until 5 seconds are over
        vTaskDelayUntil(&last_wakeup, 5000 / portTICK_PERIOD_MS);
    }
}

#elif defined(CONFIG_SHT3X_DEMO_SSLL)

/*
 * User task that triggers a measurement every 5 seconds. Due to power
 * efficiency reasons it uses *single shot* mode. In this example it starts the
 * measurement, waits for the results and fetches the results using separate
 * functions
 */
void task(void *pvParamters)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    // get the measurement duration for high repeatability;
    uint8_t duration = sht3x_get_measurement_duration(SHT3X_HIGH);

    while (1)
    {
        // Trigger one measurement in single shot mode with high repeatability.
        ESP_ERROR_CHECK(sht3x_start_measurement(&dev, SHT3X_SINGLE_SHOT, SHT3X_HIGH));

        // Wait until measurement is ready (constant time of at least 30 ms
        // or the duration returned from *sht3x_get_measurement_duration*).
        vTaskDelay(duration);

        // retrieve the values and do something with them
        ESP_ERROR_CHECK(sht3x_get_results(&dev, &temperature, &humidity));
        printf("SHT3x Sensor: %.2f °C, %.2f %%\n", temperature, humidity);

        // wait until 5 seconds are over
        vTaskDelayUntil(&last_wakeup, 5000 / portTICK_PERIOD_MS);
    }
}

#else // CONFIG_SHT3X_DEMO_PERIODIC
/*
 * User task that fetches latest measurement results of sensor every 2
 * seconds. It starts the SHT3x in periodic mode with 1 measurements per
 * second (*SHT3X_PERIODIC_1MPS*).
 */
void task(void *pvParamters)
{
    float temperature;
    float humidity;
    esp_err_t res;

    // Start periodic measurements with 1 measurement per second.
    ESP_ERROR_CHECK(sht3x_start_measurement(&dev, SHT3X_PERIODIC_1MPS, SHT3X_HIGH));

    // Wait until first measurement is ready (constant time of at least 30 ms
    // or the duration returned from *sht3x_get_measurement_duration*).
    vTaskDelay(sht3x_get_measurement_duration(SHT3X_HIGH));

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        // Get the values and do something with them.
        if ((res = sht3x_get_results(&dev, &temperature, &humidity)) == ESP_OK)
            printf("SHT3x Sensor: %.2f °C, %.2f %%\n", temperature, humidity);
        else
            printf("Could not get results: %d (%s)", res, esp_err_to_name(res));

        // Wait until 2 seconds (cycle time) are over.
        vTaskDelayUntil(&last_wakeup, 2000 / portTICK_PERIOD_MS);
    }
}

#endif

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&dev, 0, sizeof(sht3x_t));

    ESP_ERROR_CHECK(sht3x_init_desc(&dev, 0, ADDR, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(sht3x_init(&dev));

    xTaskCreatePinnedToCore(task, "sh301x_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

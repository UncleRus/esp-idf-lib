/*
 * This example code is in the Public Domain.
 */

/* standard headers */
#include <stdio.h>
#include <string.h>
#include <assert.h>

/* esp-idf headers */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <dps310.h>

#define I2C_PORT 0

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "dps310_example_default";

void dps310_task(void *pvParameters)
{
    bool sensor_ready = false;
    bool temperature_ready = false;
    bool pressure_ready = false;
    bool coef_ready = false;
    float temperature = 0;
    float pressure = 0;
    esp_err_t err = ESP_FAIL;

    /* Initialize dps310_config_t with DPS310_CONFIG_DEFAULT() macro, which
     * sets default values.
     */
    dps310_config_t config = DPS310_CONFIG_DEFAULT();

    /* Create and initialize the device descriptor with zero */
    dps310_t dev;
    memset(&dev, 0, sizeof(dps310_t));

    /* Modify dps310_config_t */
    config.tmp_oversampling = DPS310_TMP_PRC_128;
    config.pm_oversampling = DPS310_PM_PRC_128;

    /* Initialize the I2C */
    ESP_LOGI(TAG, "Initializing I2C");
    err = i2cdev_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2cdev_init(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* Initialize the device descriptor */
    ESP_LOGI(TAG, "Initializing the device descriptor");
    err = dps310_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDRESS, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_init_desc(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* Initialize the device. */
    ESP_LOGI(TAG, "Initializing the device");
    err = dps310_init(&dev, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_init(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* ensure the sensor is ready and coefficients, or COEF, are also ready.
     * */
    ESP_LOGI(TAG, "Waiting for the sensor to be ready for measurement");
    do
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!sensor_ready)
        {
            err = dps310_is_ready_for_sensor(&dev, &sensor_ready);
            if (err != ESP_OK)
            {
                goto fail;
            }
        }

        if (!coef_ready)
        {
            err = dps310_is_ready_for_coef(&dev, &coef_ready);
            if (err != ESP_OK)
            {
                goto fail;
            }
        }
    } while (!sensor_ready || !coef_ready);

    /* read COEF once, which is used to compensate the raw value. The COEF
     * values are kept in the device descriptor.
     */
    err = dps310_get_coef(&dev);
    if (err != ESP_OK)
    {
        goto fail;
    }

    ESP_LOGI(TAG, "Starting the loop");
    while (1) {

        /* Temperature command mode.
         *
         * The sensor measures temperature once, stores the raw value in a
         * resister, and sets TMP_RDY bit. The sensor goes back to standby mode
         * after measurement.
         */
        ESP_LOGI(TAG, "Setting manual temperature measurement mode");
        err = dps310_set_mode(&dev, DPS310_MODE_COMMAND_TEMPERATURE);
        if (err != ESP_OK)
        {
            goto fail;
        }

        /* wait for the result by polling TMP_RDY bit */
        ESP_LOGI(TAG, "Waiting for the temperature value to be ready");
        do
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            err = dps310_is_ready_for_temp(&dev, &temperature_ready);
            if (err != ESP_OK)
            {
                goto fail;
            }
        } while (!temperature_ready);

        /* Read the result of temperature measurement */
        err = dps310_read_temp(&dev, &temperature);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dps310_read_temp(): %s", esp_err_to_name(err));
            goto fail;
        }
        ESP_LOGI(TAG, "Temperature: %0.2f °C", temperature);

        /* Pressure command mode
         *
         * The sensor measures pressure once, stores the raw value in a resister,
         * and sets PRS_RDY bit. The sensor goes back to standby mode after
         * measurement.
         */
        ESP_LOGI(TAG, "Setting manual pressure measurement mode");
        err = dps310_set_mode(&dev, DPS310_MODE_COMMAND_PRESSURE);
        if (err != ESP_OK)
        {
            goto fail;
        }

        /* wait for the result by polling PRS_RDY bit */
        ESP_LOGI(TAG, "Waiting for the pressure value to be ready");
        do
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            err = dps310_is_ready_for_pressure(&dev, &pressure_ready);
            if (err != ESP_OK)
            {
                goto fail;
            }
        } while (!pressure_ready);

        /* Read the result of pressure measurement, and compensate the result with
         * temperature and COEF.
         *
         * Note that dps310_read_pressure() reads temperature *and* pressure
         * values for compensation, including oversampling rates.
         *
         * This implies:
         *
         * * temperature measurement must be done brefore dps310_read_pressure()
         *   at least once.
         * * the function is slow.
         */
        err = dps310_read_pressure(&dev, &pressure);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dps310_read_pressure(): %s", esp_err_to_name(err));
            goto fail;
        }
        ESP_LOGI(TAG, "Pressure: %0.2f Pa", pressure);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

fail:
    ESP_LOGE(TAG, "Halting due to error");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(dps310_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

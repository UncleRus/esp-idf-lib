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
    bool coef_ready = false;
    bool fifo_is_empty = true;
    dps310_fifo_en_mode_t fifo_en_mode = 0;
    dps310_mode_t mode = 0;
    esp_err_t err = ESP_FAIL;
    dps310_t dev;
    dps310_fifo_measurement_t measurement;

    /* Initialize dps310_config_t with DPS310_CONFIG_DEFAULT() macro, which
     * sets default values.
     */
    dps310_config_t config = DPS310_CONFIG_DEFAULT();

    /* Create and initialize the device descriptor with zero */
    memset(&dev, 0, sizeof(dps310_t));
    memset(&measurement, 0, sizeof(dps310_fifo_measurement_t));

    /* Modify dps310_config_t. Use 1 measurements per sec for temperature, 2
     * measurements per sec for pressure.
     * */
    config.tmp_oversampling = DPS310_TMP_PRC_128;
    config.pm_oversampling = DPS310_PM_PRC_128;
    config.tmp_rate  = DPS310_TMP_RATE_1;
    config.pm_rate  = DPS310_PM_RATE_2;

    /* Enable FIFO */
    config.fifo_en_mode = DPS310_FIFO_ENABLE;

    /* Initialize the I2C */
    ESP_LOGI(TAG, "Initializing I2C");
    err = i2cdev_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2cdev_init(): %s", esp_err_to_name(err));
        goto init_fail;
    }

    /* Initialize the device descriptor */
    ESP_LOGI(TAG, "Initializing the device descriptor");
    err = dps310_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDRESS, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_init_desc(): %s", esp_err_to_name(err));
        goto init_fail;
    }

    /* Initialize the device. */
    ESP_LOGI(TAG, "Initializing the device");
    err = dps310_init(&dev, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_init(): %s", esp_err_to_name(err));
        goto fail;
    }

    err = dps310_get_fifo_en(&dev, &fifo_en_mode);
    if (err != ESP_OK)
    {
        goto fail;
    }
    if (fifo_en_mode != DPS310_FIFO_ENABLE)
    {
        ESP_LOGE(TAG, "fifo_en is not enabled");
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

    /* Flush FIFO */
    ESP_LOGI(TAG, "Flushing FIFO");
    err = dps310_flush_fifo(&dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_flush_fifo(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* Temperature and pressure background mode
     *
     * The sensor continuously measures temperature and pressure. The results
     * are kept in FIFO. The FIFO has 32 slots for measurement results.
     *
     * `dps310_set_mode(&dev, DPS310_MODE_BACKGROUND_ALL)` may be used if you
     * prefer.
     */
    ESP_LOGI(TAG, "Setting background measurement mode");
    err = dps310_backgorund_start(&dev, DPS310_MODE_BACKGROUND_ALL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_mode(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_get_mode(&dev, &mode);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_get_mode(): %s", esp_err_to_name(err));
        goto fail;
    }

    if (mode != DPS310_MODE_BACKGROUND_ALL)
    {
        ESP_LOGE(TAG, "mode is not DPS310_MODE_BACKGROUND_ALL");
        goto fail;
    }

    ESP_LOGI(TAG, "Starting the loop");
    while (1) {

        vTaskDelay(pdMS_TO_TICKS(1));

        /* Skip when FIFO is empty, read FIFO when FIFO is not empty */
        err = dps310_is_fifo_empty(&dev, &fifo_is_empty);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dps310_is_fifo_empty(): %s", esp_err_to_name(err));
            goto fail;
        }
        if (fifo_is_empty)
        {
            continue;
        }

        err = dps310_read_fifo(&dev, &measurement);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dps310_read_fifo(): %s", esp_err_to_name(err));
            goto fail;

        }

        /* In the log output, there should be two pressure measurements,
         * following one temperature measurement.
         */
        switch (measurement.type)
        {
            case DPS310_MEASUREMENT_TEMPERATURE:
                ESP_LOGI(TAG, "temperature: %0.2f °C", measurement.result);
                break;
                ;;
            case DPS310_MEASUREMENT_PRESSURE:
                ESP_LOGI(TAG, "pressure: %0.2f Pa", measurement.result);
                break;
                ;;

            /* we are sure there should be a measurement result in FIFO
             * because dps310_is_fifo_empty() returned false. but when FIFO is
             * empty, the result is DPS310_MEASUREMENT_EMPTY type.
             */
            case DPS310_MEASUREMENT_EMPTY:
                ESP_LOGI(TAG, "FIFO is empty");
                break;
                ;;

            /* should not happen */
            default:
                ESP_LOGE(TAG, "Unknown dps310_fifo_measurement_type_t: %i", measurement.type);
                goto fail;
                ;;
        }
    }

fail:

    /* stop background measurement.
     *
     * ignore returned value here as it would probably fail depending on the
     * nature of the previous error, and there is nothing we can do.
     */
    ESP_LOGI(TAG, "Stopping background measurement");
    (void)dps310_backgorund_stop(&dev);

    /* free the resources.
     *
     * ignore returned value here as well.
     */
    (void)dps310_free_desc(&dev);

init_fail:
    ESP_LOGE(TAG, "Halting due to error");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(dps310_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

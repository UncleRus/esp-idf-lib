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
    float altitude = 0;
    esp_err_t err = ESP_FAIL;
    dps310_t dev;

    /* Initialize dps310_config_t with DPS310_CONFIG_DEFAULT() macro, which
     * sets default values.
     */
    dps310_config_t config = DPS310_CONFIG_DEFAULT();

    /* Create and initialize the device descriptor with zero */
    memset(&dev, 0, sizeof(dps310_t));

    config.tmp_oversampling = DPS310_TMP_PRC_64;
    config.pm_oversampling = DPS310_PM_PRC_64;
    config.tmp_rate  = DPS310_TMP_RATE_1;
    config.pm_rate  = DPS310_PM_RATE_64;

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

    /* Calibrate altitude */
    err = dps310_calibrate_altitude(&dev, CONFIG_EXAMPLE_REAL_ALTITUDE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_calibrate_altitude(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* Temperature and pressure background mode
     *
     * The sensor continuously measures temperature and pressure.
     */
    ESP_LOGI(TAG, "Setting background measurement mode");
    err = dps310_backgorund_start(&dev, DPS310_MODE_BACKGROUND_ALL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_mode(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(TAG, "Starting the loop");
    while (1) {

        vTaskDelay(pdMS_TO_TICKS(50));
        err = dps310_read_altitude(&dev, &altitude);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dps310_read_altitude(): %s", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "altitude: %0.2f (m)", altitude);
    }

fail:

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

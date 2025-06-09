/**
 * Simple example with QMI8658C sensor.
 *
 * Shows basic initialization and periodic data reading.
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
#include <qmi8658c.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "qmi8658c_example";

static void qmi8658c_task(void *pvParameters)
{
    i2c_dev_t dev = {0};

    const uint8_t I2C_ADDRESS = 0x6B;

    ESP_ERROR_CHECK(qmi8658c_init_desc(&dev, I2C_ADDRESS, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SDA));

    qmi8658c_config_t config = {
        .mode = QMI8658C_MODE_DUAL,
        .acc_scale = QMI8658C_ACC_SCALE_4G,
        .acc_odr = QMI8658C_ACC_ODR_1000,
        .gyro_scale = QMI8658C_GYRO_SCALE_64DPS,
        .gyro_odr = QMI8658C_GYRO_ODR_8000,
    };

    ESP_ERROR_CHECK(qmi8658c_setup(&dev, &config));

    vTaskDelay(pdMS_TO_TICKS(100)); // Дать сенсору стартануть

    while (1)
    {
        qmi8658c_data_t data;
        esp_err_t res = qmi8658c_read_data(&dev, &data);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Acc: x=%.3f y=%.3f z=%.3f | Gyro: x=%.3f y=%.3f z=%.3f | Temp: %.2f",
                     data.acc.x, data.acc.y, data.acc.z,
                     data.gyro.x, data.gyro.y, data.gyro.z,
                     data.temperature);
        }
        else
        {
            ESP_LOGE(TAG, "Sensor read error: %s", esp_err_to_name(res));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(qmi8658c_task, "qmi8658c_task", 4096, NULL, 5, NULL, APP_CPU_NUM);
}
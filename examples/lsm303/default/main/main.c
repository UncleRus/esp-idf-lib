#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <lsm303.h>

static const char *TAG = "IMU";

void lsm303_test(void *pvParameters)
{
    lsm303_t lsm303;

    lsm303_acc_raw_data_t acc_raw;
    lsm303_acc_data_t acc;

    lsm303_mag_raw_data_t mag_raw;
    lsm303_mag_data_t mag;

    bool acc_ready, mag_ready;

    ESP_ERROR_CHECK(lsm303_init_desc(&lsm303, LSM303_ADDR_ACC, LSM303_ADDR_MAG, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(lsm303_init(&lsm303));

    /* OPTIONAL */
    ESP_ERROR_CHECK(lsm303_acc_set_config(&lsm303, LSM303_ACC_MODE_NORMAL, LSM303_ODR_100_HZ, LSM303_ACC_SCALE_2G));
    ESP_ERROR_CHECK(lsm303_mag_set_config(&lsm303, LSM303_MAG_MODE_CONT, LSM303_MAG_RATE_15, LSM303_MAG_GAIN_1_3));

    while (1)
    {
        if (lsm303_acc_data_ready(&lsm303, &acc_ready) != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read acc");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (lsm303_mag_data_ready(&lsm303, &mag_ready) != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read mag");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (acc_ready)
        {
            if (lsm303_acc_get_raw_data(&lsm303, &acc_raw) == ESP_OK)
            {
                lsm303_acc_raw_to_g(&lsm303, &acc_raw, &acc);

                printf("Acc data: raw[x=%6d y=%6d z=%6d] x=%.2fg y=%.2fg z=%.2fg\n", acc_raw.x, acc_raw.y, acc_raw.z, acc.x, acc.y, acc.z);
            }
            else
            {
                printf("Could not read data from sensor\n");
            }
        }

        if (acc_ready)
        {
            if (lsm303_mag_get_raw_data(&lsm303, &mag_raw) == ESP_OK)
            {
                lsm303_mag_raw_to_uT(&lsm303, &mag_raw, &mag);

                printf("Mag data: raw[x=%6d y=%6d z=%6d] x=%.2fuT y=%.2fuT z=%.2fuT\n", mag_raw.x, mag_raw.y, mag_raw.z, mag.x, mag.y, mag.z);
            }
            else
            {
                printf("Could not read data from sensor\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lsm303_test, "lsm303_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

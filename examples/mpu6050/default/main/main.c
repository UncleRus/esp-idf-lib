/* standard C library */
#include <stdio.h>

/* freeRTOS library */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu6050.h"
#include <esp_err.h>
#include <esp_log.h>

#ifdef CONFIG_EXAMPLE_MPU6050_ADDRESS_HIGH
#define MPU6050_ADDR (MPU6050_ADDRESS_HIGH)
#else
#define MPU6050_ADDR (MPU6050_ADDRESS_LOW)
#endif
#define MPU6050_SDA_PIN       (CONFIG_EXAMPLE_MPU6050_I2C_MASTER_SDA)
#define MPU6050_SCL_PIN       (CONFIG_EXAMPLE_MPU6050_I2C_MASTER_SCL)
#define MPU6050_I2C_CLK_FREQZ (CONFIG_EXAMPLE_MPU6050_I2C_CLOCK_HZ)
#define MPU6050_I2C_PORT      (CONFIG_EXAMPLE_MPU6050_I2C_PORT)
#define TAG                   ("mpu6050_test")

void mpu6050_test(void *pvParameters)
{
    // testing
    esp_err_t ret = ESP_FAIL;

    mpu6050_dev_t mpu6050_conf = { 0 };

    ESP_LOGI(TAG, "mpu6050 config: addr 0x%x, sda %d, scl %d, clk, %d port %d", MPU6050_ADDR, MPU6050_SDA_PIN,
        MPU6050_SCL_PIN, MPU6050_I2C_CLK_FREQZ, MPU6050_I2C_PORT);

    while (ret != ESP_OK)
    {
        ret = mpu6050_init_desc(&(mpu6050_conf), MPU6050_ADDR, MPU6050_I2C_PORT, MPU6050_SDA_PIN, MPU6050_SCL_PIN);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to init mpu6050, error %s", esp_err_to_name(ret));
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    while (mpu6050_test_connection(&(mpu6050_conf)) != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050 connection failed.");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "mpu6050 connection successfull.");

    while (1)
    {
        mpu6050_rotation_t mpu6050_rot = { 0 };
        mpu6050_acceleration_t mpu6050_accel = { 0 };
        ret = mpu6050_get_motion(&(mpu6050_conf), &(mpu6050_accel), &(mpu6050_rot));
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "failed to read motion error %s", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG, "**********************************************************************");
            ESP_LOGI(TAG, "Rotation:\tx=%d\ty=%d\tz=%d", mpu6050_rot.x, mpu6050_rot.y, mpu6050_rot.z);
            ESP_LOGI(TAG, "Acceleration:\tx=%d\ty=%d\tz=%d", mpu6050_accel.x, mpu6050_accel.y, mpu6050_accel.z);
            ESP_LOGI(TAG, "**********************************************************************");
            int16_t temp = 0;
            ret = mpu6050_get_temperature(&(mpu6050_conf), &temp);
            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "temp %d", temp);
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // never reach here
    ESP_LOGE(TAG, "Deleting %s task", __func__);
    vTaskDelete(NULL);
}

void app_main()
{
    // task
    xTaskCreate(mpu6050_test, "mpu6050_device1", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

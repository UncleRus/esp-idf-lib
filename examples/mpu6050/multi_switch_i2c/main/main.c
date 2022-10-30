/* standard C library */
#include <stdio.h>

/* freeRTOS library */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_err.h>
#include <esp_log.h>
#include <mpu6050.h>

#define TAG ("mpu6050_test")

#define SENSOR_COUNT 2
static mpu6050_dev_t mpu6050[SENSOR_COUNT] = { 0 };

void mpu6050_test0(void *pvParameters)
{
    // testing
    esp_err_t ret = ESP_FAIL;

    ESP_LOGI(TAG, "mpu6050 config 0: addr 0x%x, sda %d, scl %d, clk, %d port %d", 0x68, 21, 22, 100000, 1);

    while (ret != ESP_OK)
    {
        ret = mpu6050_init_desc(&(mpu6050[0]), 0x68, 1, 21, 22);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to init mpu6050, error %s", esp_err_to_name(ret));
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    while (mpu6050_test_connection(&(mpu6050[0])) != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050 connection failed.");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "mpu6050 connection successfull.");

    while (1)
    {
        mpu6050_rotation_t mpu6050_rot = { 0 };
        mpu6050_acceleration_t mpu6050_accel = { 0 };
        ret = mpu6050_get_motion(&(mpu6050[0]), &(mpu6050_accel), &(mpu6050_rot));
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
            ret = mpu6050_get_temperature(&(mpu6050[0]), &temp);
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

void mpu6050_test1(void *pvParameters)
{
    // testing
    esp_err_t ret = ESP_FAIL;

    ESP_LOGI(TAG, "mpu6050 config 1: addr 0x%x, sda %d, scl %d, clk, %d port %d", 0x68, 32, 33, 100000, 2);

    while (ret != ESP_OK)
    {
        ret = mpu6050_init_desc(&(mpu6050[1]), 0x68, 2, 32, 33);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to init mpu6050, error %s", esp_err_to_name(ret));
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    while (mpu6050_test_connection(&(mpu6050[1])) != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050 connection failed.");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "mpu6050 connection successfull.");

    while (1)
    {
        mpu6050_rotation_t mpu6050_rot = { 0 };
        mpu6050_acceleration_t mpu6050_accel = { 0 };
        ret = mpu6050_get_motion(&(mpu6050[1]), &(mpu6050_accel), &(mpu6050_rot));
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
            ret = mpu6050_get_temperature(&(mpu6050[1]), &temp);
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
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(mpu6050_test0, "mpu6050_test0", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    xTaskCreate(mpu6050_test1, "mpu6050_test1", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

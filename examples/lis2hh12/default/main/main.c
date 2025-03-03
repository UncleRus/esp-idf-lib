/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, STMicroelectronics
 * Copyright (c) 2024, Ferdinand Keil <ferdinand.keil@ies.tu-darmstadt.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lis2hh12.h>
#include <string.h>
#include "esp_log.h"

#define TAG "example-lis2hh12"

#ifndef CONFIG_EXAMPLE_LIS2HH12_SA0_PIN
#define CONFIG_EXAMPLE_LIS2HH12_SA0_PIN_LEVEL false
#else
#define CONFIG_EXAMPLE_LIS2HH12_SA0_PIN_LEVEL true
#endif

#define BOOT_TIME 20  // ms
#define WAIT_TIME 100 // ms

/* Self test limits. */
#define MIN_ST_LIMIT_mg 70.0f
#define MAX_ST_LIMIT_mg 1500.0f

/* Self test results. */
#define ST_PASS 1U
#define ST_FAIL 0U

void lis2hh12_test(void *pvParameters)
{
    uint8_t whoamI;
    uint8_t rst;
    uint8_t drdy;
    uint8_t st_result;
    uint8_t i;
    uint8_t j;
    int16_t data_raw[3];
    float val_st_off[3];
    float val_st_on[3];
    float test_val[3];
    float acceleration_mg[3];

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(lis2hh12_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL, CONFIG_EXAMPLE_LIS2HH12_SA0_PIN_LEVEL));

    /* Wait sensor boot time */
    vTaskDelay(pdMS_TO_TICKS(BOOT_TIME));

    ESP_ERROR_CHECK(lis2hh12_dev_id_get(&dev, &whoamI));

    if (whoamI != LIS2HH12_ID)
    {
        ESP_LOGE(TAG, "incorrect device ID (got 0x%02x, expected 0x%02x)", whoamI, LIS2HH12_ID);
        while (1)
            ;
    }
    ESP_LOGI(TAG, "found LIS2HH12 device");

    /* Restore default configuration */
    ESP_ERROR_CHECK(lis2hh12_dev_reset_set(&dev, PROPERTY_ENABLE));

    do
    {
        ESP_ERROR_CHECK(lis2hh12_dev_reset_get(&dev, &rst));
    }
    while (rst);

    /* Enable Block Data Update */
    ESP_ERROR_CHECK(lis2hh12_block_data_update_set(&dev, PROPERTY_ENABLE));

    /*
     * Accelerometer Self Test
     */
    /* Set Output Data Rate */
    ESP_ERROR_CHECK(lis2hh12_xl_data_rate_set(&dev, LIS2HH12_XL_ODR_50Hz));
    /* Set full scale */
    ESP_ERROR_CHECK(lis2hh12_xl_full_scale_set(&dev, LIS2HH12_4g));
    /* Wait stable output */
    vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));

    /* Check if new value available */
    do
    {
        ESP_ERROR_CHECK(lis2hh12_xl_flag_data_ready_get(&dev, &drdy));
    }
    while (!drdy);

    /* Read dummy data and discard it */
    ESP_ERROR_CHECK(lis2hh12_acceleration_raw_get(&dev, data_raw));

    /* Read 5 sample and get the average vale for each axis */
    memset(val_st_off, 0x00, 3 * sizeof(float));

    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            ESP_ERROR_CHECK(lis2hh12_xl_flag_data_ready_get(&dev, &drdy));
        }
        while (!drdy);

        /* Read data and accumulate the mg value */
        ESP_ERROR_CHECK(lis2hh12_acceleration_raw_get(&dev, data_raw));

        for (j = 0; j < 3; j++)
        {
            val_st_off[j] += lis2hh12_from_fs4g_to_mg(data_raw[j]);
        }
    }

    /* Calculate the mg average values */
    for (i = 0; i < 3; i++)
    {
        val_st_off[i] /= 5.0f;
    }

    /* Enable Self Test positive (or negative) */
    ESP_ERROR_CHECK(lis2hh12_xl_self_test_set(&dev, LIS2HH12_ST_NEGATIVE));
    /* Wait stable output */
    vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));

    /* Check if new value available */
    do
    {
        ESP_ERROR_CHECK(lis2hh12_xl_flag_data_ready_get(&dev, &drdy));
    }
    while (!drdy);

    /* Read dummy data and discard it */
    ESP_ERROR_CHECK(lis2hh12_acceleration_raw_get(&dev, data_raw));
    /* Read 5 sample and get the average vale for each axis */
    memset(val_st_on, 0x00, 3 * sizeof(float));

    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            ESP_ERROR_CHECK(lis2hh12_xl_flag_data_ready_get(&dev, &drdy));
        }
        while (!drdy);

        /* Read data and accumulate the mg value */
        ESP_ERROR_CHECK(lis2hh12_acceleration_raw_get(&dev, data_raw));

        for (j = 0; j < 3; j++)
        {
            val_st_on[j] += lis2hh12_from_fs4g_to_mg(data_raw[j]);
        }
    }

    /* Calculate the mg average values */
    for (i = 0; i < 3; i++)
    {
        val_st_on[i] /= 5.0f;
    }

    /* Calculate the mg values for self test */
    for (i = 0; i < 3; i++)
    {
        test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
    }

    /* Check self test limit */
    st_result = ST_PASS;

    for (i = 0; i < 3; i++)
    {
        if ((MIN_ST_LIMIT_mg > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_mg))
        {
            st_result = ST_FAIL;
        }
    }

    /* Disable Self Test */
    ESP_ERROR_CHECK(lis2hh12_xl_self_test_set(&dev, LIS2HH12_ST_DISABLE));
    /* Disable sensor. */
    ESP_ERROR_CHECK(lis2hh12_xl_data_rate_set(&dev, LIS2HH12_XL_ODR_OFF));

    if (st_result == ST_PASS)
    {
        ESP_LOGI(TAG, "Self Test - PASS");
    }
    else
    {
        ESP_LOGI(TAG, "Self Test - FAIL");
    }

    /* Enable Block Data Update */
    ESP_ERROR_CHECK(lis2hh12_block_data_update_set(&dev, PROPERTY_ENABLE));
    /* Set full scale */
    ESP_ERROR_CHECK(lis2hh12_xl_full_scale_set(&dev, LIS2HH12_8g));
    /* Configure filtering chain */
    /* Accelerometer data output- filter path / bandwidth */
    ESP_ERROR_CHECK(lis2hh12_xl_filter_aalias_bandwidth_set(&dev, LIS2HH12_AUTO));
    ESP_ERROR_CHECK(lis2hh12_xl_filter_out_path_set(&dev, LIS2HH12_FILT_LP));
    ESP_ERROR_CHECK(lis2hh12_xl_filter_low_bandwidth_set(&dev, LIS2HH12_LP_ODR_DIV_400));
    /* Accelerometer interrupt - filter path / bandwidth */
    ESP_ERROR_CHECK(lis2hh12_xl_filter_int_path_set(&dev, LIS2HH12_HP_DISABLE));
    /* Set Output Data Rate */
    ESP_ERROR_CHECK(lis2hh12_xl_data_rate_set(&dev, LIS2HH12_XL_ODR_100Hz));

    /*  Read samples in polling mode (no int) */
    while (1)
    {
        uint8_t reg;
        /*  Read output only if new value is available */
        ESP_ERROR_CHECK(lis2hh12_xl_flag_data_ready_get(&dev, &reg));

        if (reg)
        {
            /* Read acceleration data  */
            memset(data_raw, 0x00, 3 * sizeof(int16_t));
            ESP_ERROR_CHECK(lis2hh12_acceleration_raw_get(&dev, data_raw));
            acceleration_mg[0] = lis2hh12_from_fs8g_to_mg(data_raw[0]);
            acceleration_mg[1] = lis2hh12_from_fs8g_to_mg(data_raw[1]);
            acceleration_mg[2] = lis2hh12_from_fs8g_to_mg(data_raw[2]);
            ESP_LOGI(TAG, "Acceleration [mg]:\t%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
        }

        vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
    }

    vTaskDelete(NULL);
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(lis2hh12_test, "lis2hh12_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

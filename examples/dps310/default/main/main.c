/*
 * Copyright (c) 2022 Tomoyuki Sakurai <y@trombik.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
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
    esp_err_t err = ESP_FAIL;
    uint8_t reg_value = 0;
    dps310_t dev;
    dps310_config_t config = DPS310_CONFIG_DEFAULT();

    ESP_LOGI(TAG, "Initializing I2C");
    err = i2cdev_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2cdev_init(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(TAG, "Initializing the device descriptor");
    err = dps310_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDRESS, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_init_desc(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(TAG, "Initializing the device");
    err = dps310_init(&dev, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_init(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(TAG, "Get Pressure measurement rate");
    err = dps310_get_pm_rate(&dev, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_pm_rate(): %s", esp_err_to_name(err));
        goto fail;
    }
    assert(reg_value == DPS310_PM_RATE_1);

    ESP_LOGI(TAG, "Set Pressure measurement rate to DPS310_PM_RATE_16");
    err = dps310_set_pm_rate(&dev, DPS310_PM_RATE_16);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_pm_rate(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(TAG, "Get Pressure measurement rate");
    err = dps310_get_pm_rate(&dev, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_pm_rate(): %s", esp_err_to_name(err));
        goto fail;
    }
    assert(reg_value == DPS310_PM_RATE_16);

    ESP_LOGI(TAG, "Get Pressure oversampling rate");
    err = dps310_get_pm_prc(&dev, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_pm_prc(): %s", esp_err_to_name(err));
        goto fail;
    }
    assert(reg_value == DPS310_TMP_PRC_1);

    ESP_LOGI(TAG, "Set Pressure oversampling rate to DPS310_TMP_PRC_16");
    err = dps310_set_pm_prc(&dev, DPS310_TMP_PRC_16);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_pm_prc(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(TAG, "Get Pressure oversampling rate");
    err = dps310_get_pm_prc(&dev, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_pm_prc(): %s", esp_err_to_name(err));
        goto fail;
    }
    ESP_LOGI(TAG, "reg_value: %d", reg_value);
    assert(reg_value == DPS310_TMP_PRC_16);

    ESP_LOGI(TAG, "Starting the loop");
    while (1) {
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

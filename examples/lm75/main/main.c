/*
 * Copyright (c) 2019 Tomoyuki Sakurai <y@trombik.org>
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

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <string.h>
#include <esp_log.h>
#include <lm75.h>

#define I2C_PORT 0

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "lm75_example";

void lm75_task(void *pvParameters)
{
    uint8_t v;
    int shutdown = 0;
    float temperature;
    float os_temperature = 25.0;
    float os_temperature_in_reg;
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    /* device operation mode: shutdown
     * overtemperature polarity: high
     * overtemperature mode: interrupt
     * overtemperature fault queue: 4
     */
    lm75_config_t config = {
        .mode       = LM75_MODE_SHUTDOWN,
        .os_pol     = LM75_OSP_HIGH,
        .os_mode    = LM75_OS_MODE_INT,
        .os_fault_queue = LM75_FAULT_QUEUE4
    };

    ESP_LOGI(TAG, "Initializing I2C");
    ESP_ERROR_CHECK(i2cdev_init());

    ESP_LOGI(TAG, "Initializing LM75 descriptor");
    ESP_ERROR_CHECK(lm75_init_desc(&dev, LM75_I2C_ADDRESS_DEFAULT, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    ESP_LOGI(TAG, "Initializing LM75");
    ESP_ERROR_CHECK(lm75_init(&dev, config));

    ESP_LOGI(TAG, "Wakeup LM75");
    ESP_ERROR_CHECK(lm75_wakeup(&dev));

    ESP_LOGI(TAG, "Read default Overtemperature Shutdown temperature");
    ESP_ERROR_CHECK(lm75_get_os_threshold(&dev, &os_temperature_in_reg));

    /* float is used in format string.
     * for ESP8266, make sure sdkconfig has the following line:
     *
     * CONFIG_NEWLIB_LIBRARY_LEVEL_NORMAL=y
     *
     * this is enabled by default for this example.
     */
    ESP_LOGI(TAG, "OS temperature: %f", os_temperature_in_reg);

    ESP_LOGI(TAG, "Set Overtemperature Shutdown temperature to %f", os_temperature);
    ESP_ERROR_CHECK(lm75_set_os_threshold(&dev, os_temperature));
    ESP_LOGI(TAG, "Read Overtemperature Shutdown temperature");
    ESP_ERROR_CHECK(lm75_get_os_threshold(&dev, &os_temperature_in_reg));
    if (os_temperature_in_reg != os_temperature) {
        ESP_LOGE(TAG, "invalid OS temperature found: %.3f", os_temperature_in_reg);
    }

    ESP_LOGI(TAG, "Set OS polarity to LM75_OSP_HIGH");
    ESP_ERROR_CHECK(lm75_set_os_polarity(&dev, LM75_OSP_HIGH));
    ESP_ERROR_CHECK(lm75_get_os_polarity(&dev, &v));
    if (v != LM75_OSP_HIGH) {
        ESP_LOGE(TAG, "polarity is not LM75_OSP_HIGH");
    }

    ESP_LOGI(TAG, "Set OS polarity to LM75_OSP_LOW");
    ESP_ERROR_CHECK(lm75_set_os_polarity(&dev, LM75_OSP_LOW));
    ESP_ERROR_CHECK(lm75_get_os_polarity(&dev, &v));
    if (v != LM75_OSP_LOW) {
        ESP_LOGE(TAG, "polarity is not LM75_OSP_LOW");
    }

    ESP_LOGI(TAG, "Starting the loop");
    while (1) {
        shutdown ^= 1;
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (shutdown) {
            ESP_ERROR_CHECK(lm75_shutdown(&dev));
        } else {
            ESP_ERROR_CHECK(lm75_wakeup(&dev));
        }

        /* When in normal mode, the output should have a bit of deviation.
         * When in shutdown mode, the output should be same.
         */
        printf("Operation mode: %s\n", shutdown ? "shutdown" : "normal");
        for (int i = 0; i < 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (lm75_read_temperature(&dev, &temperature) != ESP_OK) {
                ESP_LOGE(TAG, "failed to read_temperature()");
                continue;
            }
            printf("Temperature: %.3f\n", temperature);
        }
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(lm75_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

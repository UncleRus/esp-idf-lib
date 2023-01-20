/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2022 Marc Luehr <marcluehr@gmail.com>
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <driver/i2c.h>

#include <veml7700.h>
#include <string.h>

static const char *TAG = "VEML7700main";

#define SDA_GPIO_NUM CONFIG_EXAMPLE_SDA_GPIO_NUM
#define SCL_GPIO_NUM CONFIG_EXAMPLE_SCL_GPIO_NUM

static bool compare_configuration(veml7700_config_t* config_a, veml7700_config_t* config_b)
{
    return (config_a->gain == config_b->gain) &&
        (config_a->gain == config_b->gain) &&
        (config_a->integration_time == config_b->integration_time) &&
        (config_a->persistence_protect == config_b->persistence_protect) &&
        (config_a->interrupt_enable == config_b->interrupt_enable) &&
        (config_a->shutdown == config_b->shutdown) &&
        (config_a->threshold_high == config_b->threshold_high) &&
        (config_a->threshold_low == config_b->threshold_low) &&
        (config_a->power_saving_mode == config_b->power_saving_mode) &&
        (config_a->power_saving_enable == config_b->power_saving_enable);
}

void app_main(void)
{
    i2c_dev_t veml7700_device;
    veml7700_config_t veml7700_configuration;
    
    memset(&veml7700_device, 0, sizeof(i2c_dev_t));
    memset(&veml7700_configuration, 0, sizeof(veml7700_config_t));
    
    ESP_LOGI(TAG, "initializing hardware");

    // remember to always init i2cdev first before using it
    ESP_ERROR_CHECK(i2cdev_init());

    // initialize the device struct
    ESP_ERROR_CHECK(veml7700_init_desc(&veml7700_device, I2C_NUM_0, SDA_GPIO_NUM, SCL_GPIO_NUM));

#ifdef CONFIG_EXAMPLE_I2C_MASTER_SDA
    veml7700_device.cfg.sda_pullup_en = true;
#endif

#ifdef CONFIG_EXAMPLE_I2C_MASTER_SCL
    veml7700_device.cfg.scl_pullup_en = true;
#endif
    // check if the device is available
    ESP_ERROR_CHECK(veml7700_probe(&veml7700_device));

    /* set configuration parameters
     * select gain 1/8 coastest resolution but the sensor will most likely not
     * over-saturated
     */
    veml7700_configuration.gain = VEML7700_GAIN_DIV_8;

    /* set the time to integrate the light values. more time will lead to a finer
     * resolution but will be over-saturated earlier
     */
    veml7700_configuration.integration_time = VEML7700_INTEGRATION_TIME_100MS;

    // interrupt is not used
    veml7700_configuration.persistence_protect = VEML7700_PERSISTENCE_PROTECTION_4;
    veml7700_configuration.interrupt_enable = 1;
    veml7700_configuration.shutdown = 0;

    /* set power saving modes. This reduces the repeated measurement. When disabled
     * the sensors performs one measurement cycle after the other (here: 100ms 
     * intergration time) setting the power save mode add a 1000ms sleep during 
     * measurements.
     */
    veml7700_configuration.power_saving_mode = VEML7700_POWER_SAVING_MODE_1000MS;
    veml7700_configuration.power_saving_enable = 1;

    // write the configuration to the device
    ESP_ERROR_CHECK(veml7700_set_config(&veml7700_device, &veml7700_configuration));

    /* read back the configuration for testing purposes. This step is not needed
     * for a normal application
     */
    veml7700_config_t veml7700_configuration_readback;
    ESP_ERROR_CHECK(veml7700_get_config(&veml7700_device, &veml7700_configuration_readback));

    if (compare_configuration(&veml7700_configuration_readback, 
        &veml7700_configuration_readback))
    {
        ESP_LOGI(TAG, "Configuration read back matches");
    }
    else
    {
        ESP_LOGE(TAG, "Configuration read back does not match");
    }

    while (1)
    {
        uint32_t als = 0;
        uint32_t white = 0;
        bool low_threshold = false;
        bool high_threshold = false;
        
        veml7700_get_ambient_light(&veml7700_device, &veml7700_configuration, &als);
        veml7700_get_white_channel(&veml7700_device, &veml7700_configuration, &white);
        veml7700_get_white_channel(&veml7700_device, &veml7700_configuration, &white);
        veml7700_get_interrupt_status(&veml7700_device, &low_threshold, 
            &high_threshold);
        
        ESP_LOGI(TAG, "ALS: %u lx", als);
        ESP_LOGI(TAG, "WHITE: %u lx", white);
        ESP_LOGI(TAG, "Interrupt status: low: %d high: %d", low_threshold, 
            high_threshold);
        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}

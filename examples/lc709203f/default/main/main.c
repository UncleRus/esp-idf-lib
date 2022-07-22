#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_log.h>

#if CONFIG_EXAMPLE_I2C_ONBOARD_PULLUP_GPIO_OUTPUT > -1
#include <driver/gpio.h>
#endif

#include "lc709203f.h"

static const char *TAG = "Exammple";

static esp_err_t initialize_lc709203f(i2c_dev_t *lc)
{
    if (!lc)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(lc709203f_set_power_mode(lc, LC709203F_POWER_MODE_OPERATIONAL));
    // Using 2500mAh LiPo battery. Check Datasheet graph for APA values by battery type & mAh
    ESP_ERROR_CHECK(lc709203f_set_apa(lc, 0x32));
    ESP_ERROR_CHECK(lc709203f_set_battery_profile(lc, LC709203F_BATTERY_PROFILE_1));
    ESP_ERROR_CHECK(lc709203f_initial_rsoc(lc));
    ESP_ERROR_CHECK(lc709203f_set_temp_mode(lc, LC709203F_TEMP_MODE_I2C));
    ESP_ERROR_CHECK(lc709203f_set_cell_temperature_celsius(lc, 20));

    uint16_t value = 0;
    ESP_ERROR_CHECK(lc709203f_get_power_mode(lc, (lc709203f_power_mode_t *)&value));
    ESP_LOGI(TAG, "Power Mode: 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_apa(lc, (uint8_t *)&value));
    ESP_LOGI(TAG, "APA: 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_battery_profile(lc, (lc709203f_battery_profile_t *)&value));
    ESP_LOGI(TAG, "Battery Profile: 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_temp_mode(lc, (lc709203f_temp_mode_t *)&value));
    ESP_LOGI(TAG, "Temp Mode: 0x%X", value);

    return ESP_OK;
}

void lc709203f_test(void *pvParameters)
{
    i2c_dev_t lc;
    uint16_t voltage = 0, rsoc = 0, ite = 0;
    float temperature = -274;

    memset(&lc, 0, sizeof(lc));

    ESP_ERROR_CHECK(lc709203f_init_desc(&lc, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    initialize_lc709203f(&lc);

    while (1)
    {
        ESP_ERROR_CHECK(lc709203f_get_cell_voltage(&lc, &voltage));
        ESP_ERROR_CHECK(lc709203f_get_rsoc(&lc, &rsoc));
        ESP_ERROR_CHECK(lc709203f_get_cell_ite(&lc, &ite));
        // Temperature in I2C mode. Temperature should be the same as configured.
        ESP_ERROR_CHECK(lc709203f_get_cell_temperature_celsius(&lc, &temperature));
        ESP_LOGI(TAG, "Temp: %.1f\tVoltage: %.2f\tRSOC: %d%%\tITE: %.1f%%", temperature, voltage / 1000.0, rsoc,
            ite / 10.0);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
#if CONFIG_EXAMPLE_I2C_ONBOARD_PULLUP_GPIO_OUTPUT > -1
    /// Adafruit Feather esp32ss/s3 needs to set GPIO7 as HIGH level output to enable onboard I2C pull ups
    /// We needn't internal pull ups.
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1 << CONFIG_EXAMPLE_I2C_ONBOARD_PULLUP_GPIO_OUTPUT);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(CONFIG_EXAMPLE_I2C_ONBOARD_PULLUP_GPIO_OUTPUT, CONFIG_EXAMPLE_I2C_ONBOARD_PULLUP_GPIO_OUTPUT_LEVEL);
#endif

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lc709203f_test, "lc709203f_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
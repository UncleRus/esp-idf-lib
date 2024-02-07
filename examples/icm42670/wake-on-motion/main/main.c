#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <icm42670.h>

static const char *TAG = "icm42670";

#define PORT 0
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_GND)
#define I2C_ADDR ICM42670_I2C_ADDR_GND
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_VCC)
#define I2C_ADDR ICM42670_I2C_ADDR_VCC
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

/* Find gpio definitions in sdkconfig */

void icm42670_wom_test(void *pvParameters)
{
    // config IO0 as input, pull-up enabled
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT(CONFIG_EXAMPLE_INT_INPUT_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    // init device descriptor and device
    icm42670_t dev = { 0 };
    ESP_ERROR_CHECK(icm42670_init_desc(&dev, I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    /* config a Wake-On-Motion (WoM) interrupt on ICM42670-pin 2
     * - interrupt pin on ICM42670 = 2
     * - signal level is latched
     * - signal is fully driven (push/pull)
     * - polarity is active high
    */ 
    const uint8_t int_pin = 2;
    const icm42670_int_config_t int_config = {
        .mode = ICM42670_INT_MODE_LATCHED,
        .drive = ICM42670_INT_DRIVE_PUSH_PULL,
        .polarity = ICM42670_INT_POLARITY_ACTIVE_HIGH,
    };
    ESP_ERROR_CHECK(icm42670_config_int_pin(&dev, int_pin, int_config));

    // enable interrupt sources (in this case all three axes)
    icm42670_int_source_t sources = {false};
    sources.wom_z = true;
    sources.wom_y = true;
    sources.wom_z = true;
    ESP_ERROR_CHECK(icm42670_set_int_sources(&dev, int_pin, sources));

    /* configure Wake-On-Motion (WoM):
     * - first exceeding of the threshold is considered as WoM event
     * - the WoM event sources are logically linked by a OR
     * - the reference measurement for the threshold is taken at startup
     * - the threshold is set to 100, which corresponds to 0.39*g 
     *      (WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]
     *      Resolution 1g/256=~3.9 mg)
     */
    const icm42670_wom_config_t wom_config = {
        .trigger = ICM42670_WOM_INT_DUR_FIRST,
        .logical_mode = ICM42670_WOM_INT_MODE_ALL_OR,
        .reference = ICM42670_WOM_MODE_REF_INITIAL,
        .wom_y_threshold = 100,
        .wom_z_threshold = 100,
        .wom_x_threshold = 100,
    };
    ESP_ERROR_CHECK(icm42670_config_wom(&dev, wom_config));

    // set output-data-rate (ODR) and averaging (AVG) on accelerometer
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_accel_avg(&dev, ICM42670_ACCEL_AVG_8X));

    // disable gyro and enable accelerometer in low-power (LP) mode
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_DISABLE));
    ESP_ERROR_CHECK(icm42670_set_low_power_clock(&dev, ICM42670_LP_CLK_WUO));
    ESP_ERROR_CHECK_WITHOUT_ABORT(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LP_MODE));

    //enable WoM
    ESP_ERROR_CHECK(icm42670_enable_wom(&dev, true));

    // now poll intterupt pin for changes
    while(1)
    {
        ESP_LOGI(TAG, "WoM event detected: %s", gpio_get_level(CONFIG_EXAMPLE_INT_INPUT_PIN) ? "true" : "false");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    
    xTaskCreatePinnedToCore(icm42670_wom_test, "icm42670_wom_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


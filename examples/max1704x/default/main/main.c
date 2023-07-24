#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <max1704x.h>
#include <esp_err.h>
#include <esp_log.h>

static char *TAG = "main";

void test(void *pvParameters)
{
    esp_err_t r;
    max1704x_t dev = {0 };
    max1704x_config_t config = { 0 };
    max1704x_status_t status = { 0 };
    uint16_t version = 0;
    float voltage = 0;
    float soc_percent = 0;
    float rate_change = 0;
    
    /**
     * Set up I2C bus to communicate with MAX1704X
     */

    dev.model = MAX17043_4;

    ESP_ERROR_CHECK(max1704x_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(max1704x_quickstart(&dev));
    ESP_ERROR_CHECK(max1704x_get_version(&dev, &version));
    ESP_LOGI(TAG, "Version: %d\n", version);

    /**
     * Get MAX1704X configuration
     */ 
    ESP_LOGI(TAG, "--- MAX1704X config register ---");
    ESP_ERROR_CHECK(max1704x_get_config(&dev));
    ESP_LOGI(TAG, "Alert Status: %d", dev.config.alert_status);
    ESP_LOGI(TAG, "Sleep Mode: %d", dev.config.sleep_mode);
    ESP_LOGI(TAG, "SOC Change Alert Mode: %d", dev.config.soc_change_alert);
    ESP_LOGI(TAG, "Empty Alert Threshold: %d%%", dev.config.empty_alert_thresh);
    ESP_LOGI(TAG, "RCOMP Value: %d (%x)", dev.config.rcomp, dev.config.rcomp);
    ESP_LOGI(TAG, "--- End Configuration ---\n");

    // Change configuration settings
    config.soc_change_alert = true;
    config.empty_alert_thresh = 10;
    ESP_LOGI(TAG, "Setting new MAX1704X configuration");
    ESP_ERROR_CHECK(max1704x_set_config(&dev, &config));

    ESP_LOGI(TAG, "--- MAX1704X config register after updating configurations ---");
    ESP_ERROR_CHECK(max1704x_get_config(&dev));
    ESP_LOGI(TAG, "Alert Status: %d", dev.config.alert_status);
    ESP_LOGI(TAG, "Sleep Mode: %d", dev.config.sleep_mode);
    ESP_LOGI(TAG, "SOC Change Alert Mode: %d", dev.config.soc_change_alert);
    ESP_LOGI(TAG, "Empty Alert Threshold: %d%%", dev.config.empty_alert_thresh);
    ESP_LOGI(TAG, "RCOMP Value: %d (%x)", dev.config.rcomp, dev.config.rcomp);
    ESP_LOGI(TAG, "--- End Configuration ---\n");

    /**
     * Get current MAX1704X status
     */
    ESP_LOGI(TAG, "--- MAX1704X status register ---");
    ESP_ERROR_CHECK(max1704x_get_status(&dev));
    ESP_LOGI(TAG, "Reset Indicator: %d", dev.status.reset_indicator);
    ESP_LOGI(TAG, "Voltage High Alert: %d", dev.status.voltage_high);
    ESP_LOGI(TAG, "Voltage Low Alert: %d", dev.status.voltage_low);
    ESP_LOGI(TAG, "Voltage Reset Alert: %d", dev.status.voltage_reset);
    ESP_LOGI(TAG, "SOC Low Alert: %d", dev.status.soc_low);
    ESP_LOGI(TAG, "SOC Change Alert: %d", dev.status.soc_change);
    ESP_LOGI(TAG, "Voltage Reset Alert Enabled: %d", dev.status.vreset_alert);
    ESP_LOGI(TAG, "--- End Status ---\n");

    // Update MAX1704X status register to clear the reset indicator
    ESP_LOGI(TAG, "Setting the status register to clear reset indicator");
    status.reset_indicator = false;

    ESP_ERROR_CHECK(max1704x_set_status(&dev, &status));
    ESP_LOGI(TAG, "--- MAX1704X status register after updating status register ---");
    ESP_ERROR_CHECK(max1704x_get_status(&dev));
    ESP_LOGI(TAG, "Reset Indicator: %d", dev.status.reset_indicator);
    ESP_LOGI(TAG, "Voltage High Alert: %d", dev.status.voltage_high);
    ESP_LOGI(TAG, "Voltage Low Alert: %d", dev.status.voltage_low);
    ESP_LOGI(TAG, "Voltage Reset Alert: %d", dev.status.voltage_reset);
    ESP_LOGI(TAG, "SOC Low Alert: %d", dev.status.soc_low);
    ESP_LOGI(TAG, "SOC Change Alert: %d", dev.status.soc_change);
    ESP_LOGI(TAG, "Voltage Reset Alert Enabled: %d", dev.status.vreset_alert);
    ESP_LOGI(TAG, "--- End Status ---\n");

    /**
     * Get current MAX1704X voltage, SOC, and rate of change every 5 seconds
     */

    while (1)
    {
        r = max1704x_get_voltage(&dev, &voltage);

        if (r == ESP_OK) {
            ESP_LOGI(TAG, "Voltage: %.2fV", voltage);
        }
        else
            ESP_LOGI(TAG, "Error %d: %s", r, esp_err_to_name(r));

        r = max1704x_get_soc(&dev, &soc_percent);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "SOC: %.2f%%", soc_percent);
        }
        else
            ESP_LOGI(TAG, "Error %d: %s", r, esp_err_to_name(r));

        r = max1704x_get_crate(&dev, &rate_change);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "SOC rate of change: %.2f%%", rate_change);
        }
        else
            ESP_LOGI(TAG, "Error %d: %s", r, esp_err_to_name(r));
        
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main()
{
    /**
     * Initialize I2C bus
     */
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <max1704x.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

static char *tag = "main";

void test(void *pvParameters)
{
    i2c_dev_t dev; 
    esp_err_t r;
    max1704x_t max1704x;
    max1704x_config_t config;
    max1704x_status_t status;
    uint16_t version = 0;
    float voltage = 0;
    float soc_percent = 0;
    float rate_change = 0;
    
    memset(&dev, 0, sizeof(i2c_dev_t));
    memset(&max1704x, 0, sizeof(max1704x_t));
    memset(&config, 0, sizeof(max1704x_config_t));
    memset(&status, 0, sizeof(max1704x_status_t));

    /**
     * Set up I2C bus to communicate with MAX1704X
     */

    ESP_ERROR_CHECK(max1704x_init_desc(&dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(max1704x_init(&max1704x, &dev));
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_ERROR_CHECK(max1704x_quickstart(&max1704x));
    ESP_ERROR_CHECK(max1704x_get_version(&max1704x, &version));
    ESP_LOGI(tag, "Version: %d\n", version);

    /**
     * Get MAX1704X configuration
     */ 
    ESP_LOGI(tag, "--- MAX1704X config register ---");
    ESP_ERROR_CHECK(max1704x_get_config(&max1704x));
    ESP_LOGI(tag, "Alert Status: %d", max1704x.config.alert_status);
    ESP_LOGI(tag, "Sleep Mode: %d", max1704x.config.sleep_mode);
    ESP_LOGI(tag, "SOC Change Alert Mode: %d", max1704x.config.soc_change_alert);
    ESP_LOGI(tag, "Empty Alert Threshold: %d%%", max1704x.config.empty_alert_thresh);
    ESP_LOGI(tag, "RCOMP Value: %d (%x)", max1704x.config.rcomp, max1704x.config.rcomp);
    ESP_LOGI(tag, "--- End Configuration ---\n");

    // Change configuration settings
    config.soc_change_alert = true;
    config.empty_alert_thresh = 10;
    ESP_LOGI(tag, "Setting new MAX1704X configuration");
    ESP_ERROR_CHECK(max1704x_set_config(&max1704x, &config));

    ESP_LOGI(tag, "--- MAX1704X config register after updating configurations ---");
    ESP_ERROR_CHECK(max1704x_get_config(&max1704x));
    ESP_LOGI(tag, "Alert Status: %d", max1704x.config.alert_status);
    ESP_LOGI(tag, "Sleep Mode: %d", max1704x.config.sleep_mode);
    ESP_LOGI(tag, "SOC Change Alert Mode: %d", max1704x.config.soc_change_alert);
    ESP_LOGI(tag, "Empty Alert Threshold: %d%%", max1704x.config.empty_alert_thresh);
    ESP_LOGI(tag, "RCOMP Value: %d (%x)", max1704x.config.rcomp, max1704x.config.rcomp);
    ESP_LOGI(tag, "--- End Configuration ---\n");

    /**
     * Get current MAX1704X status
     */
    ESP_LOGI(tag, "--- MAX1704X status register ---");
    ESP_ERROR_CHECK(max1704x_get_status(&max1704x));
    ESP_LOGI(tag, "Reset Indicator: %d", max1704x.status.reset_indicator);
    ESP_LOGI(tag, "Voltage High Alert: %d", max1704x.status.voltage_high);
    ESP_LOGI(tag, "Voltage Low Alert: %d", max1704x.status.voltage_low);
    ESP_LOGI(tag, "Voltage Reset Alert: %d", max1704x.status.voltage_reset);
    ESP_LOGI(tag, "SOC Low Alert: %d", max1704x.status.soc_low);
    ESP_LOGI(tag, "SOC Change Alert: %d", max1704x.status.soc_change);
    ESP_LOGI(tag, "Voltage Reset Alert Enabled: %d", max1704x.status.vreset_alert);
    ESP_LOGI(tag, "--- End Status ---\n");

    // Update MAX1704X status register to clear the reset indicator
    ESP_LOGI(tag, "Setting the status register to clear reset indicator");
    status.reset_indicator = false;

    ESP_ERROR_CHECK(max1704x_set_status(&max1704x, &status));
    ESP_LOGI(tag, "--- MAX1704X status register after updating status register ---");
    ESP_ERROR_CHECK(max1704x_get_status(&max1704x));
    ESP_LOGI(tag, "Reset Indicator: %d", max1704x.status.reset_indicator);
    ESP_LOGI(tag, "Voltage High Alert: %d", max1704x.status.voltage_high);
    ESP_LOGI(tag, "Voltage Low Alert: %d", max1704x.status.voltage_low);
    ESP_LOGI(tag, "Voltage Reset Alert: %d", max1704x.status.voltage_reset);
    ESP_LOGI(tag, "SOC Low Alert: %d", max1704x.status.soc_low);
    ESP_LOGI(tag, "SOC Change Alert: %d", max1704x.status.soc_change);
    ESP_LOGI(tag, "Voltage Reset Alert Enabled: %d", max1704x.status.vreset_alert);
    ESP_LOGI(tag, "--- End Status ---\n");

    /**
     * Get current MAX1704X voltage, SOC, and rate of change every 5 seconds
     */

    while (1)
    {
        r = max1704x_get_voltage(&max1704x, &voltage);

        if (r == ESP_OK) {
            ESP_LOGI(tag, "Voltage: %.2fV", voltage);
        }
        else
            ESP_LOGI(tag, "Error %d: %s", r, esp_err_to_name(r));

        r = max1704x_get_soc(&max1704x, &soc_percent);
        if (r == ESP_OK) {
            ESP_LOGI(tag, "SOC: %.2f%%", soc_percent);
        }
        else
            ESP_LOGI(tag, "Error %d: %s", r, esp_err_to_name(r));

        r = max1704x_get_crate(&max1704x, &rate_change);
        if (r == ESP_OK) {
            ESP_LOGI(tag, "SOC rate of change: %.2f%%", rate_change);
        }
        else
            ESP_LOGI(tag, "Error %d: %s", r, esp_err_to_name(r));
        
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main()
{
    /**
     * Initialize I2C bus
     */
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}


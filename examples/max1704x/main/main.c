#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <max1704x.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

#define I2C_MASTER_SDA_IO EXAMPLE_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO EXAMPLE_I2C_MASTER_SCL
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

static char *tag = "main";

void test(void *pvParameters)
{
    i2c_dev_t dev; 
    esp_err_t r;
    max1704x_t max1704x;
    uint16_t version = 0;
    float voltage = 0;
    float soc_percent = 0;
    float rate_change = 0;
    
    memset(&dev, 0, sizeof(i2c_dev_t));
    memset(&max1704x, 0, sizeof(max1704x_t));

    ESP_ERROR_CHECK(max1704x_init_desc(&dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(max1704x_init(&max1704x, &dev, MAX17048_9));
    ESP_ERROR_CHECK(max1704x_quickstart(&max1704x));
    ESP_ERROR_CHECK(max1704x_get_version(&max1704x, &version));
    ESP_LOGI(tag, "Version: %d\n", version);

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
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

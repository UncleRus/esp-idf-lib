#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_idf_version.h>
#include <max31855.h>

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define HOST    HSPI_HOST
#else
#define HOST    SPI2_HOST
#endif

static const char *TAG = "max31855-example";

static void task(void *pvParameter)
{
    max31855_t dev = { 0 };
    // Configure SPI bus
    spi_bus_config_t cfg = {
       .mosi_io_num = -1,
       .miso_io_num = CONFIG_EXAMPLE_MISO_GPIO,
       .sclk_io_num = CONFIG_EXAMPLE_SCLK_GPIO,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    // Init device
    ESP_ERROR_CHECK(max31855_init_desc(&dev, HOST, MAX31855_MAX_CLOCK_SPEED_HZ, CONFIG_EXAMPLE_CS_GPIO));

    float tc_t, cj_t;
    bool scv, scg, oc;
    while (1)
    {
        esp_err_t res = max31855_get_temperature(&dev, &tc_t, &cj_t, &scv, &scg, &oc);
        if (res != ESP_OK)
            ESP_LOGE(TAG, "Failed to measure: %d (%s)", res, esp_err_to_name(res));
        else
        {
            if (scv) ESP_LOGW(TAG, "Thermocouple shorted to VCC!");
            if (scg) ESP_LOGW(TAG, "Thermocouple shorted to GND!");
            if (oc) ESP_LOGW(TAG, "No connection to thermocouple!");
            ESP_LOGI(TAG, "Temperature: %.2f°C, cold junction temperature: %.4f°C", tc_t, cj_t);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main()
{
    xTaskCreate(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}


#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_idf_version.h>
#include <max31865.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define HOST    HSPI_HOST
#else
#define HOST    SPI2_HOST
#endif

#if CONFIG_EXAMPLE_CONN_2WIRE
#define RTD_CONNECTION MAX31865_2WIRE
#endif
#if CONFIG_EXAMPLE_CONN_3WIRE
#define RTD_CONNECTION MAX31865_3WIRE
#endif
#if CONFIG_EXAMPLE_CONN_4WIRE
#define RTD_CONNECTION MAX31865_4WIRE
#endif

#if CONFIG_EXAMPLE_SCALE_ITS90
#define RTD_STANDARD MAX31865_ITS90
#endif
#if CONFIG_EXAMPLE_SCALE_DIN43760
#define RTD_STANDARD MAX31865_DIN43760
#endif
#if CONFIG_EXAMPLE_SCALE_US
#define RTD_STANDARD MAX31865_US_INDUSTRIAL
#endif

#if CONFIG_EXAMPLE_FILTER_50
#define FILTER MAX31865_FILTER_50HZ
#endif
#if CONFIG_EXAMPLE_FILTER_60
#define FILTER MAX31865_FILTER_60HZ
#endif

static const char *TAG = "max31865-example";

static max31865_config_t config = {
    .v_bias = true,
    .filter = FILTER,
    .mode = MAX31865_MODE_SINGLE,
    .connection = RTD_CONNECTION
};

static void task(void *pvParameter)
{
    // Configure SPI bus
    spi_bus_config_t cfg = {
       .mosi_io_num = CONFIG_EXAMPLE_MOSI_GPIO,
       .miso_io_num = CONFIG_EXAMPLE_MISO_GPIO,
       .sclk_io_num = CONFIG_EXAMPLE_SCLK_GPIO,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    // Init device
    max31865_t dev = {
        .standard = RTD_STANDARD,
        .r_ref = CONFIG_EXAMPLE_RTD_REF,
        .rtd_nominal = CONFIG_EXAMPLE_RTD_NOMINAL,
    };
    ESP_ERROR_CHECK(max31865_init_desc(&dev, HOST, MAX31865_MAX_CLOCK_SPEED_HZ, CONFIG_EXAMPLE_CS_GPIO));

    // Configure device
    ESP_ERROR_CHECK(max31865_set_config(&dev, &config));

    float temperature;
    while (1)
    {
        esp_err_t res = max31865_measure(&dev, &temperature);
        if (res != ESP_OK)
            ESP_LOGE(TAG, "Failed to measure: %d (%s)", res, esp_err_to_name(res));
        else
            ESP_LOGI(TAG, "Temperature: %.4f", temperature);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


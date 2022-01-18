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

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define RTD_STANDARD MAX31865_ITS90   // ITS-90
#define RTD_NOMINAL 100               // 100 Ohm, PT100
#define RTD_REFERENCE 430             // Rref = 430 Ohm
#define RTD_CONNECTION MAX31865_2WIRE // 2-wire connection configuration

static const char *TAG = "max31865-example";

static max31865_config_t config = {
    .v_bias = true,
    .filter = MAX31865_FILTER_50HZ,
    .mode = MAX31865_MODE_SINGLE,
    .connection = RTD_CONNECTION
};

static void task(void *pvParameter)
{
    esp_err_t res;

    // Configure SPI bus
    spi_bus_config_t cfg = {
       .mosi_io_num = PIN_NUM_MOSI,
       .miso_io_num = PIN_NUM_MISO,
       .sclk_io_num = PIN_NUM_CLK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    // Init device
    max31865_t dev = {
        .standard = RTD_STANDARD,
        .r_ref = RTD_REFERENCE,
        .rtd_nominal = RTD_NOMINAL,
    };
    ESP_ERROR_CHECK(max31865_init_desc(&dev, HOST, PIN_NUM_CS));

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


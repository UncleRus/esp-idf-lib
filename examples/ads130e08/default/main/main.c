#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "freertos/semphr.h"
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_idf_version.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>
#include <ads130e08.h>

#if CONFIG_IDF_TARGET_ESP32
#define HOST HSPI_HOST
#elif CONFIG_IDF_TARGET_ESP32S3
#define HOST SPI2_HOST
#endif

static const char *TAG_ADS130E08 = "ads130e08";

ads130e08_t adc_dev;
ads130e08_raw_data_t raw_data;

SemaphoreHandle_t ads130e08_drdy_semaphore = NULL;

void IRAM_ATTR gpio_isr_handler(void *arg)
{
    /* data ready */
    xSemaphoreGiveFromISR(ads130e08_drdy_semaphore, NULL);
    portYIELD_FROM_ISR();
}

// Main task
void ads130e08_test(void *pvParameters)
{
    // Initialize semaphore
    ads130e08_drdy_semaphore = xSemaphoreCreateBinary();

    // Configure SPI bus
    spi_bus_config_t spi_cfg = {
        .mosi_io_num = CONFIG_EXAMPLE_MOSI_GPIO,
        .miso_io_num = CONFIG_EXAMPLE_MISO_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &spi_cfg, 1));

    // Init device
    static ads130e08_dev_config_t dev_config = {
        .clk_en = ADS130E08_CLK_OUT_DISABLED,
        .int_test = ADS130E08_INT_TEST_EXTERNAL,
        .test_amp = ADS130E08_TEST_AMP_CALIB_1X,
        .test_freq = ADS130E08_TEST_FREQ_EXP_21,
        .pd_refbuf = ADS130E08_INTERNAL_REF_BUFFER_ENABLED,
        .vref_4v = ADS130E08_REF_VOLTAGE_2_4V,
        .opamp_ref = ADS130E08_NON_INVERTING_CONNECT_OPAMP,
        .pd_opamp = ADS130E08_OPAMP_DISABLED,
    };

    ESP_ERROR_CHECK(ads130e08_init_desc(&adc_dev, HOST, CONFIG_EXAMPLE_CS_GPIO));
    ESP_ERROR_CHECK(ads130e08_send_system_cmd(&adc_dev, ADS130E08_CMD_STOP));
    ESP_ERROR_CHECK(ads130e08_send_system_cmd(&adc_dev, ADS130E08_CMD_RESET));
    ESP_ERROR_CHECK(ads130e08_send_data_read_cmd(&adc_dev, ADS130E08_CMD_SDATAC));
    ESP_ERROR_CHECK(ads130e08_set_device_config(&adc_dev, dev_config));

    // Configure ISR
    gpio_config_t int_ads130e08 = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_EXAMPLE_INT_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&int_ads130e08));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_EXAMPLE_INT_GPIO, gpio_isr_handler, (void *)CONFIG_EXAMPLE_INT_GPIO));

    // Start conversion
    ESP_ERROR_CHECK(ads130e08_send_system_cmd(&adc_dev, ADS130E08_CMD_START));

    for (;;)
    {
        xSemaphoreTake(ads130e08_drdy_semaphore, portMAX_DELAY);

        ESP_ERROR_CHECK(ads130e08_get_rdata(&adc_dev, &raw_data));

        ESP_LOGI(TAG_ADS130E08, "Fault status -> Positive %02x, Negative %02x", raw_data.fault_statp,
            raw_data.fault_statn);
        ESP_LOGI(TAG_ADS130E08, "Gpios level -> %02x", raw_data.gpios_level);
        ESP_LOGI(TAG_ADS130E08, "Raw data -> CH1: %d CH2: %d CH3: %d CH4: %d CH5: %d CH6: %d CH7: %d CH8: %d",
            raw_data.channels_raw[0], raw_data.channels_raw[1], raw_data.channels_raw[2], raw_data.channels_raw[3],
            raw_data.channels_raw[4], raw_data.channels_raw[5], raw_data.channels_raw[6], raw_data.channels_raw[7]);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(ads130e08_test, "ads130e08_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_log.h>
#include <ssd1306.h>
#include <driver/spi_master.h>
#include "image.xbm"
#include "sdkconfig.h"

static const char *TAG = "ssd1306_spi4_example";

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define HOST VSPI_HOST
#else
#define HOST SPI3_HOST
#endif
#define MOSI_GPIO 23
#define CLK_GPIO  18
#define CS_GPIO   16
#define DC_GPIO   17

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static void task(void *arg)
{
    spi_bus_config_t cfg = {
       .mosi_io_num = MOSI_GPIO,
       .miso_io_num = -1,
       .sclk_io_num = CLK_GPIO,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    ssd1306_t display;
    memset(&display, 0, sizeof(display));
    display.chip = SSD1306_CHIP;
    display.width = DISPLAY_WIDTH;
    display.height = DISPLAY_HEIGHT;

    ESP_ERROR_CHECK(ssd1306_init_desc(&display, HOST, CS_GPIO, DC_GPIO));

    ESP_ERROR_CHECK(ssd1306_init(&display));

    ssd1306_clear(&display);
    ssd1306_load_xbm(&display, image_bits);

    // send frambuffer to display
    ssd1306_flush(&display);


    while (1)
    {
        ESP_LOGI(TAG, "wait...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    // Start task
    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


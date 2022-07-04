#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_version.h>
#include <max7219.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define HOST    HSPI_HOST
#else
#define HOST    SPI2_HOST
#endif

void task(void *pvParameter)
{
    // Configure SPI bus
    spi_bus_config_t cfg = {
       .mosi_io_num = CONFIG_EXAMPLE_PIN_NUM_MOSI,
       .miso_io_num = -1,
       .sclk_io_num = CONFIG_EXAMPLE_PIN_NUM_CLK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    // Configure device
    max7219_t dev = {
       .cascade_size = 1,
       .digits = 8,
       .mirrored = true
    };
    ESP_ERROR_CHECK(max7219_init_desc(&dev, HOST, MAX7219_MAX_CLOCK_SPEED_HZ, CONFIG_EXAMPLE_PIN_NUM_CS));

    char buf[10]; // 8 digits + decimal point + \0

    ESP_ERROR_CHECK(max7219_init(&dev));

    while (1)
    {
        printf("Display cycle\n");

        max7219_clear(&dev);
        max7219_draw_text_7seg(&dev, 0, "7219LEDS");
        vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_DELAY));

        max7219_clear(&dev);
        sprintf(buf, "%2.4f A", 34.6782);
        max7219_draw_text_7seg(&dev, 0, buf);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_DELAY));

        max7219_clear(&dev);
        sprintf(buf, "%08x", 12345678);
        max7219_draw_text_7seg(&dev, 0, buf);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_DELAY));
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(task, "task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);
}


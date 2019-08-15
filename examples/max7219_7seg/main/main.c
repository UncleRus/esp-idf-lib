#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <max7219.h>

#define DELAY 2000

#define HOST HSPI_HOST

#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define CHECK(expr, msg) \
    while ((res = expr) != ESP_OK) { \
        printf(msg "\n", res); \
        vTaskDelay(250 / portTICK_RATE_MS); \
    }


void task(void *pvParameter)
{
    esp_err_t res;

    // Configure SPI bus
    spi_bus_config_t cfg = {
       .mosi_io_num = PIN_NUM_MOSI,
       .miso_io_num = -1,
       .sclk_io_num = PIN_NUM_CLK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    CHECK(spi_bus_initialize(HOST, &cfg, 1),
            "Could not initialize SPI bus: %d");

    // Configure device
    max7219_t dev = {
       .cascade_size = 1,
       .digits = 8,
       .mirrored = true
    };
    CHECK(max7219_init_desc(&dev, HOST, PIN_NUM_CS),
            "Could not initialize MAX7129 descriptor: %d");

    char buf[10];

    CHECK(max7219_init(&dev),
            "Could not initialize MAX7129: %d");

    while (1)
    {
        printf("Display cycle\n");

        max7219_clear(&dev);
        max7219_draw_text_7seg(&dev, 0, "7219LEDS");
        vTaskDelay(DELAY / portTICK_PERIOD_MS);

        max7219_clear(&dev);
        sprintf(buf, "%2.4f A", 34.6782);
        max7219_draw_text_7seg(&dev, 0, buf);
        vTaskDelay(DELAY / portTICK_PERIOD_MS);

        max7219_clear(&dev);
        sprintf(buf, "%08x", 12345678);
        max7219_draw_text_7seg(&dev, 0, buf);
        vTaskDelay(DELAY / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(task, "task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);
}


#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <max7219.h>

#define SCROLL_DELAY 50
#define CASCADE_SIZE 1

#define HOST HSPI_HOST

#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define CHECK(expr, msg) \
    while ((res = expr) != ESP_OK) { \
        printf(msg "\n", res); \
        vTaskDelay(250 / portTICK_RATE_MS); \
    }

static const uint64_t symbols[] = {
    0x383838fe7c381000, // arrows
    0x10387cfe38383800,
    0x10307efe7e301000,
    0x1018fcfefc181000,
    0x10387cfefeee4400, // heart
    0x105438ee38541000, // sun

    0x7e1818181c181800, // digits
    0x7e060c3060663c00,
    0x3c66603860663c00,
    0x30307e3234383000,
    0x3c6660603e067e00,
    0x3c66663e06663c00,
    0x1818183030667e00,
    0x3c66663c66663c00,
    0x3c66607c66663c00,
    0x3c66666e76663c00
};
const static size_t symbols_size = sizeof(symbols) - sizeof(uint64_t) * CASCADE_SIZE;

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
       .cascade_size = CASCADE_SIZE,
       .digits = 0,
       .mirrored = true
    };
    CHECK(max7219_init_desc(&dev, HOST, PIN_NUM_CS),
            "Could not initialize MAX7129 descriptor: %d");
    CHECK(max7219_init(&dev),
            "Could not initialize MAX7129: %d");
    size_t offs = 0;
    while (1)
    {
        printf("---------- draw\n");

        for (uint8_t c = 0; c < CASCADE_SIZE; c ++)
            max7219_draw_image_8x8(&dev, c, (uint8_t *)symbols + c * 8 + offs);
        vTaskDelay(SCROLL_DELAY / portTICK_PERIOD_MS);

        if (++ offs == symbols_size)
            offs = 0;
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(task, "task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);
}


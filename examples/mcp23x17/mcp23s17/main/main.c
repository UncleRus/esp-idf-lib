#include <inttypes.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mcp23x17.h>
#include <driver/gpio.h>
#include <string.h>

#define HOST SPI2_HOST

static const char *TAG = "mcp23s17 example";

static mcp23x17_t dev = { 0 };

static void intr_handler(void *arg)
{
    uint32_t v;
    mcp23x17_get_level(&dev, 0, &v);
    ESP_LOGI(TAG, "Interrupt! PORTA0: %" PRIu32, v);
}

void test(void *pvParameters)
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
    ESP_ERROR_CHECK(mcp23x17_init_desc_spi(&dev, HOST, MCP23X17_MAX_SPI_FREQ, CONFIG_EXAMPLE_ADDRESS, CONFIG_EXAMPLE_CS_GPIO));

    // Set INTx mode to active high
    ESP_ERROR_CHECK(mcp23x17_set_int_out_mode(&dev, MCP23X17_ACTIVE_HIGH));
    // Setup PORTA0 as input
    ESP_ERROR_CHECK(mcp23x17_set_mode(&dev, 0, MCP23X17_GPIO_INPUT));
    // Enable pull-up
    ESP_ERROR_CHECK(mcp23x17_set_pullup(&dev, 0, true));
    // Setup interrupt on it
    ESP_ERROR_CHECK(mcp23x17_set_interrupt(&dev, 0, MCP23X17_INT_ANY_EDGE));

    // Setup INTA GPIO interrupt
    gpio_set_direction(CONFIG_EXAMPLE_INTA_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(CONFIG_EXAMPLE_INTA_GPIO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_EXAMPLE_INTA_GPIO, intr_handler, NULL);

    // Setup PORTB0 as output
    mcp23x17_set_mode(&dev, 8, MCP23X17_GPIO_OUTPUT);
    // do some blinking
    bool on = true;
    while (1)
    {
        mcp23x17_set_level(&dev, 8, on);
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}

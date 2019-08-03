#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mcp23x17.h>
#include <driver/gpio.h>
#include <string.h>

// A0, A1, A2 pins are grounded

#define INTA_GPIO 19  // INTA pin
#define SDA_GPIO 16
#define SCL_GPIO 17

static void IRAM_ATTR intr_handler(void *arg)
{
    printf("Interrupt!\n");
}

void test(void *pvParameters)
{
    mcp23x17_t dev;
    memset(&dev, 0, sizeof(mcp23x17_t));

    ESP_ERROR_CHECK(mcp23x17_init_desc(&dev, 0, MCP23X17_ADDR_BASE, SDA_GPIO, SCL_GPIO));

    // Setup PORTA0 as input
    mcp23x17_set_mode(&dev, 0, MCP23X17_GPIO_INPUT);
    // Setup interrupt on it
    mcp23x17_set_interrupt(&dev, 0, MCP23X17_INT_ANY_EDGE);

    gpio_set_direction(INTA_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(INTA_GPIO, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTA_GPIO, intr_handler, NULL);

    // Setup PORTB0 as output
    mcp23x17_set_mode(&dev, 8, MCP23X17_GPIO_OUTPUT);
    // do some blinkning
    bool on = true;
    while (1)
    {
        mcp23x17_set_level(&dev, 8, on);
        on = !on;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}


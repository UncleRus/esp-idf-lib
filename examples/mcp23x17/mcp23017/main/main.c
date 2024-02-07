#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <mcp23x17.h>
#include <driver/gpio.h>

static const char *TAG = "mcp23017_example";

static EventGroupHandle_t eg = NULL;
static mcp23x17_t dev = { 0 };

#define BIT_BUTTON_CHANGED BIT(0)

static void IRAM_ATTR intr_handler(void *arg)
{
    // On interrupt set bit in event group
    BaseType_t hp_task;
    if (xEventGroupSetBitsFromISR(eg, BIT_BUTTON_CHANGED, &hp_task) != pdFAIL)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        portYIELD_FROM_ISR(hp_task);
#else
        portYIELD_FROM_ISR();
#endif
}

void button_handler(void *pvParameters)
{
    while (1)
    {
        // wait for BIT_BUTTON_CHANGED, clear it on exit
        if (xEventGroupWaitBits(eg, BIT_BUTTON_CHANGED, pdTRUE, pdTRUE, portMAX_DELAY) != BIT_BUTTON_CHANGED)
            continue;
        // OK, we got this bit set
        ESP_LOGI(TAG, "Button was pressed!");
    }
}

void test(void *pvParameters)
{
    eg = xEventGroupCreate();

    ESP_ERROR_CHECK(mcp23x17_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Setup PORTA0 as input
    mcp23x17_set_mode(&dev, 0, MCP23X17_GPIO_INPUT);
    // Setup interrupt on it
    mcp23x17_set_interrupt(&dev, 0, MCP23X17_INT_ANY_EDGE);

    // Run button handler
    xTaskCreate(button_handler, "button_handler", 4096, NULL, 5, NULL);

    // Setup GPIO interrupt
    gpio_set_direction(CONFIG_EXAMPLE_INTA_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(CONFIG_EXAMPLE_INTA_GPIO, GPIO_INTR_ANYEDGE);
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
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}

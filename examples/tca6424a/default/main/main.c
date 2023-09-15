#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <tca6424a.h>
#include <driver/gpio.h>
#include <inttypes.h>
#include <esp_log.h>

static i2c_dev_t tca6424a = {0 };

static QueueHandle_t gpio_queue = NULL;

static const char *TAG = "tca6424a-example";

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_GND
#define ADDR TCA6424A_I2C_ADDRESS_GND
#endif
#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_VCC
#define ADDR TCA6424A_I2C_ADDRESS_VCC
#endif

// Interrupt handler
static void IRAM_ATTR intr_handler(void *arg)
{
    gpio_num_t gpio = (gpio_num_t)arg;
    xQueueSendFromISR(gpio_queue, &gpio, NULL);
}

// Interrupt event receiver
static void gpio_recv_task(void *arg)
{
    gpio_num_t gpio;
    while (1)
    {
        if (xQueueReceive(gpio_queue, &gpio, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "GPIO interrupt on pin %d", gpio);
            if (gpio != CONFIG_EXAMPLE_INT_GPIO) continue;

            uint32_t val;
            esp_err_t res = tca6424a_port_read(&tca6424a, &val);
            if (res != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading TCA6424A: %d (%s)", res, esp_err_to_name(res));
                continue;
            }

            ESP_LOGI(TAG, "TCA6424A port value: 0x%" PRIX32, val);
        }
    }
}

void main_task(void *pvParameters)
{
    // Init descriptor
    ESP_ERROR_CHECK(tca6424a_init_desc(&tca6424a, ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Setup P00, P01 and P02 as input, others as output
    ESP_ERROR_CHECK(tca6424a_port_set_mode(&tca6424a, 0x000007)); // 0b000000000000000000000111

    // Create queue
    gpio_queue = xQueueCreate(5, sizeof(gpio_num_t));

    // Run event receiver
    xTaskCreate(gpio_recv_task, "gpio_recv_task", 4096, NULL, 5, NULL);

    // Setup GPIO interrupt
    gpio_set_direction(CONFIG_EXAMPLE_INT_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(CONFIG_EXAMPLE_INT_GPIO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_EXAMPLE_INT_GPIO, intr_handler, (void *)CONFIG_EXAMPLE_INT_GPIO);

    // blink on P10
    bool on = true;
    while (1)
    {
        ESP_ERROR_CHECK(tca6424a_set_level(&tca6424a, 8, on));
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(main_task, "main_task", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}

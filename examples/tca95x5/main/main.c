#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tca95x5.h>
#include <driver/gpio.h>
#include <string.h>

// A0, A1, A2 pins are grounded

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define INT_GPIO 16
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define INT_GPIO 19
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

static void IRAM_ATTR intr_handler(void *arg)
{
    printf("Interrupt!\n");
}

void test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(tca95x5_init_desc(&dev, 0, TCA95X5_I2C_ADDR_BASE, SDA_GPIO, SCL_GPIO));

    // Setup P00, P01 and P02 as input, other as output
    ESP_ERROR_CHECK(tca95x5_port_set_mode(&dev, 7)); // 0b0000000000000111
    // Setup interrupt
    gpio_set_direction(INT_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(INT_GPIO, GPIO_INTR_LOW_LEVEL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT_GPIO, intr_handler, NULL);

    // blink on P10
    bool on = true;
    while (1)
    {
        ESP_ERROR_CHECK(tca95x5_set_level(&dev, 8, on));
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}


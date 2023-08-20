#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds3502.h>
#include <string.h>
#include <esp_log.h>

#if defined(CONFIG_EXAMPLE_DS3502_ADDR_0)
#define ADDR DS3502_ADDR_0
#endif
#if defined(CONFIG_EXAMPLE_DS3502_ADDR_1)
#define ADDR DS3502_ADDR_1
#endif
#if defined(CONFIG_EXAMPLE_DS3502_ADDR_2)
#define ADDR DS3502_ADDR_2
#endif
#if defined(CONFIG_EXAMPLE_DS3502_ADDR_3)
#define ADDR DS3502_ADDR_3
#endif

static const char *TAG = "ds3502_test";

void test(void *pvParameters)
{
    i2c_dev_t dev;

    // Zero device descriptor
    memset(&dev, 0, sizeof(i2c_dev_t));

    // Initialize i2c device descriptor
    ESP_ERROR_CHECK(ds3502_init_desc(&dev, 0, ADDR, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // Initialize potentiometer
    ESP_ERROR_CHECK(ds3502_init(&dev));

    uint8_t pos = 0;
    // Read wiper position
    ESP_ERROR_CHECK(ds3502_get(&dev, &pos));

    // Change resistance up and down
    uint8_t dir = 1;
    while (1)
    {
        // Set wiper position
        ESP_ERROR_CHECK(ds3502_set(&dev, pos, false));

        ESP_LOGE(TAG, "Wiper position: %d, R: %0.2f kOhm", pos, 10.0 / 128 * pos);

        // Increment/decrement position
        if (pos == DS3502_MAX)
            dir = -1;
        else if (pos == 0)
            dir = 1;
        pos += dir;

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}


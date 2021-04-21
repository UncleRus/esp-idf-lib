#include "sdkconfig.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_log.h>
#include <ssd1306.h>
#include "image.xbm"

static const char *TAG = "ssd1306_i2c_example";

#define I2C_PORT 0
#define ADDR SSD1306_I2C_ADDR0
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static void task(void *arg)
{
    ssd1306_t display;
    memset(&display, 0, sizeof(display));
    display.chip = SSD1306_CHIP;
    display.width = DISPLAY_WIDTH;
    display.height = DISPLAY_HEIGHT;

    ESP_ERROR_CHECK(ssd1306_init_desc(&display, I2C_PORT, ADDR, SDA_GPIO, SCL_GPIO));
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
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());

    // Start task
    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}


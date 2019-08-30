#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <pcf8574.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif
#define I2C_ADDR 0x27

static i2c_dev_t pcf8574;

static esp_err_t write_lcd_data(uint8_t data)
{
    return pcf8574_port_write(&pcf8574, data);
}

void lcd_test(void *pvParameters)
{
    hd44780_t lcd = {
        .write_cb = write_lcd_data, // use callback to send data to LCD by I2C GPIO expander
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = 0,
            .e  = 2,
            .d4 = 4,
            .d5 = 5,
            .d6 = 6,
            .d7 = 7,
            .bl = 3
        }
    };

    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, 0, I2C_ADDR, SDA_GPIO, SCL_GPIO));

    ESP_ERROR_CHECK(hd44780_init(&lcd));

    hd44780_switch_backlight(&lcd, true);

    uint8_t direction = 1;
    while (1) {
        direction ^= 1;
        hd44780_gotoxy(&lcd, 0, 0);
        hd44780_puts(&lcd, direction ? "LEFT scroll " : "RIGHT scroll");

        for (uint8_t i = 0; i < 16; i++) {
            if (direction) {
                hd44780_scroll_left(&lcd);
            } else {
                hd44780_scroll_right(&lcd);
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lcd_test, "lcd_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}


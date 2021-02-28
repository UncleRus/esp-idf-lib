#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <pcf8574.h>
#include <bmp280.h>
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

static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data)
{
    return pcf8574_port_write(&pcf8574, data);
}

void lcd_test(void *pvParameters)
{
    // Fill LCD descriptor
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

    // Prepare PCF8574
    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, 0, I2C_ADDR, SDA_GPIO, SCL_GPIO));

    // Prepare BMP280
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));
    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    // Init screen, switch backlight on
    ESP_ERROR_CHECK(hd44780_init(&lcd));
    hd44780_switch_backlight(&lcd, true);

    char s[16];
    float pressure, temperature;

    while (1)
    {
        // read sensor
        if (bmp280_read_float(&dev, &temperature, &pressure, NULL) != ESP_OK)
        {
            hd44780_clear(&lcd);
            hd44780_puts(&lcd, "Failed!");
        }
        else
        {
            // print data
            snprintf(s, sizeof(s), "t: %.01f C", temperature);
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, s);

            snprintf(s, sizeof(s), "P: %.02f kPa", pressure / 1000);
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, s);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lcd_test, "lcd_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}


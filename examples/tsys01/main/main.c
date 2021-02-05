#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <tsys01.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif
#define I2C_PORT 0

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void task(void *arg)
{
    tsys01_t dev;
    memset(&dev, 0, sizeof(tsys01_t));

    ESP_ERROR_CHECK(tsys01_init_desc(&dev, TSYS01_I2C_ADDR1, I2C_PORT, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(tsys01_init(&dev));

    printf("Device serial number: 0x%06x\n", dev.serial);

    ESP_ERROR_CHECK(tsys01_reset(&dev));

    float t;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_ERROR_CHECK(tsys01_measure(&dev, &t));
        printf("Temperature: %.02f deg.C\n", t);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tsys01.h>

#if CONFIG_EXAMPLE_I2C_ADDRESS1
#define ADDRESS TSYS01_I2C_ADDR1
#else
#define ADDRESS TSYS01_I2C_ADDR2
#endif

#define I2C_PORT 0

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void task(void *arg)
{
    tsys01_t dev = { 0 };

    ESP_ERROR_CHECK(tsys01_init_desc(&dev, ADDRESS, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(tsys01_init(&dev));

    printf("Device serial number: 0x%06" PRIx32 "\n", dev.serial);

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

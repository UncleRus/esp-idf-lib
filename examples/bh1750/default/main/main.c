#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bh1750.h>
#include <string.h>

#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_LO)
#define ADDR BH1750_ADDR_LO
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_HI)
#define ADDR BH1750_ADDR_HI
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t)); // Zero descriptor

    ESP_ERROR_CHECK(bh1750_init_desc(&dev, ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(bh1750_setup(&dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH));

    while (1)
    {
        uint16_t lux;

        if (bh1750_read(&dev, &lux) != ESP_OK)
            printf("Could not read lux data\n");
        else
            printf("Lux: %d\n", lux);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init()); // Init library

    xTaskCreatePinnedToCore(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL, APP_CPU_NUM);
}


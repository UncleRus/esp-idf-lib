#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <qmc5883l.h>
#include <string.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void qmc5883l_test(void *pvParameters)
{
    qmc5883l_t dev;

    memset(&dev, 0, sizeof(qmc5883l_t));

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(qmc5883l_init_desc(&dev, QMC5883L_I2C_ADDR_DEF, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // 50Hz data rate, 128 samples, -2G..+2G range
    ESP_ERROR_CHECK(qmc5883l_set_config(&dev, QMC5883L_DR_50, QMC5883L_OSR_128, QMC5883L_RNG_2));

    while (1)
    {
        qmc5883l_data_t data;
        if (qmc5883l_get_data(&dev, &data) == ESP_OK)
            /* float is used in printf(). you need non-default configuration in
             * sdkconfig for ESP8266, which is enabled by default for this
             * example. see sdkconfig.defaults.esp8266
             */
            printf("Magnetic data: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", data.x, data.y, data.z);
        else
            printf("Could not read QMC5883L data\n");

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(qmc5883l_test, "qmc5883l_test", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, APP_CPU_NUM);
}


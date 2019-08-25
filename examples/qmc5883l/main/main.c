#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <qmc5883l.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

static void wait_for_data(qmc5883l_t *dev)
{
    while (true)
    {
        bool ready;
        ESP_ERROR_CHECK(qmc5883l_data_ready(dev, &ready));
        if (ready)
            return;
        vTaskDelay(1);
    }
}

void qmc5883l_test(void *pvParameters)
{
    qmc5883l_t dev;

    memset(&dev, 0, sizeof(qmc5883l_t));

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(qmc5883l_init_desc(&dev, 0, QMC5883L_I2C_ADDR_DEF, SDA_GPIO, SCL_GPIO));

    // 50Hz data rate, 128 samples, -2G..+2G range
    ESP_ERROR_CHECK(qmc5883l_set_config(&dev, QMC5883L_DR_50, QMC5883L_OSR_128, QMC5883L_RNG_2));

    while (1)
    {
        wait_for_data(&dev);

        qmc5883l_data_t data;
        if (qmc5883l_get_data(&dev, &data) == ESP_OK)
            /* float is used in printf(). you need non-default configuration in
             * sdkconfig for ESP8266, which is enabled by default for this
             * example. see sdkconfig.defaults.esp8266
             */
            printf("Magnetic data: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", data.x, data.y, data.z);
        else
            printf("Could not read QMC5883L data\n");

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreatePinnedToCore(qmc5883l_test, "qmc5883l_test", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, APP_CPU_NUM);
}


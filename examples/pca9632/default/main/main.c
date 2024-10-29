#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <pca9632.h>

void pca9632_test(void *pvParameters)
{
    i2c_dev_t pca9632;

    ESP_ERROR_CHECK(pca9632_init_desc(&pca9632, PCA9632_I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(pca9632_init(&pca9632));
    pca9632_set_output_params(&pca9632, true, true);
    pca9632_debug(&pca9632);

    pca9632_set_pwm_all(&pca9632, 0x00, 0x00, 0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(5000));

    pca9632_set_pwm_all(&pca9632, 0xff, 0x00, 0x00, 0x00);
    pca9632_debug(&pca9632);
    vTaskDelay(pdMS_TO_TICKS(3000));

    pca9632_set_pwm_all(&pca9632, 0x00, 0xff, 0x00, 0x00);
    pca9632_debug(&pca9632);
    vTaskDelay(pdMS_TO_TICKS(3000));

    pca9632_set_pwm_all(&pca9632, 0x00, 0x00, 0xff, 0x00);
    pca9632_debug(&pca9632);
    vTaskDelay(pdMS_TO_TICKS(3000));

    pca9632_set_pwm_all(&pca9632, 0x00, 0x00, 0x00, 0xff);
    pca9632_debug(&pca9632);
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1)

        for (pca9632_led_t led = 0; led <= LED3; led++)
        {
            printf("PCA9632 LED%d\n", led);
            pca9632_debug(&pca9632);
            for (int i = 0; i < 0xff; i++)
            {
                pca9632_set_pwm(&pca9632, led, i);
                printf("PCA9632 LED%d val=%d\n", led, i);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            pca9632_set_pwm(&pca9632, led, 0);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    printf("Hello ESP\n");
    xTaskCreate(pca9632_test, "pca9632", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

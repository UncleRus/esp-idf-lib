#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include <stdbool.h>
#include "driver/gpio.h"
#include <i2cdev.h>

#include <bmi160.h>

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_GND
#define ADDR BMI160_I2C_ADDRESS_GND
#else
#define ADDR BMI160_I2C_ADDRESS_VDD
#endif

#define I2C_PORT 0
#define WARNING_CHANNEL 1
#define WARNING_CURRENT (40.0)

static const char *TAG = "BMI160_EXAMPLE";

/* Example for regular pooling or interrupt readout */

#define USE_NEW_DATA_INT // uncomment if you want to use interrupt for new data

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 2.44Mhz

TaskHandle_t xTaskHandlePtr = NULL;

#ifdef USE_NEW_DATA_INT
static void IRAM_ATTR isr_new_data(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xTaskHandlePtr, &xHigherPriorityTaskWoken);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
    portYIELD_FROM_ISR();
#endif
}
#endif


void bmi160_test(void *pvParameters)
{
    
    bmi160_t bmi160_dev;
    memset(&bmi160_dev.i2c_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(i2cdev_init());

#ifdef USE_NEW_DATA_INT
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_INT1_GPIO);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //install gpio isr service
#ifdef CONFIG_IDF_TARGET_ESP8266
    gpio_install_isr_service(1<<10u);
#else
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
#endif
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(CONFIG_EXAMPLE_INT1_GPIO, isr_new_data, NULL);
#endif
    ESP_LOGI(TAG,"Example for data read out \n");

    ESP_LOGI(TAG,"Initializing BMI160\n");

    ESP_ERROR_CHECK(bmi160_init(&bmi160_dev, ADDR, I2C_PORT, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO));

    ESP_ERROR_CHECK(bmi160_self_test(&bmi160_dev));

    bmi160_conf_t bmi160_conf = {
        .accRange = BMI160_ACC_RANGE_2G,
        .accOdr = BMI160_ACC_ODR_200HZ,
        .accAvg = BMI160_ACC_LP_AVG_2,
        .accMode = BMI160_PMU_ACC_NORMAL,
        .gyrRange = BMI160_GYR_RANGE_125DPS,
        .gyrOdr = BMI160_GYR_ODR_200HZ,
        .gyrMode = BMI160_PMU_GYR_NORMAL,
        .accUs = 0u
    };
    
    ESP_ERROR_CHECK(bmi160_start(&bmi160_dev, &bmi160_conf));

    ESP_ERROR_CHECK(bmi160_calibrate(&bmi160_dev));

#ifdef USE_NEW_DATA_INT
    //enable interrupt on bmi160
    bmi160_int_out_conf_t intOutConf = {
        .intPin = BMI160_PIN_INT1,
        .intEnable = BMI160_INT_ENABLE,
        .intOd = BMI160_INT_PUSH_PULL,
        .intLevel = BMI160_INT_ACTIVE_HIGH
    };
    ESP_ERROR_CHECK(bmi160_enable_int_new_data(&bmi160_dev, &intOutConf));
#endif

    while(1)
    {

    #ifdef USE_NEW_DATA_INT
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        bmi160_result_t result;
        esp_err_t ret = bmi160_read_data(&bmi160_dev, &result);
        if(ret == ESP_OK)
        {
            ESP_LOGI(TAG,"%+.3f %+.3f %+.3f %+.3f %+.3f %+.3f\n", result.accX, result.accY, result.accZ, result.gyroX, result.gyroY, result.gyroZ);
        }
        else
        {
            ESP_LOGI(TAG,"No new data\n");
        }
    #else
        bmi160_result_t result;
        esp_err_t ret = bmi160_read_data(&bmi160_dev, &result);
        if(ret == ESP_OK)
        {
            //print all data in format with 3 decimal places
            ESP_LOGI(TAG,"Accel: %+.3f %+.3f %+.3f Gyro: %+.3f %+.3f %+.3f\n", result.accX, result.accY, result.accZ, result.gyroX, result.gyroY, result.gyroZ);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    #endif
    }

    ESP_ERROR_CHECK(bmi160_free(&bmi160_dev));
}


void app_main(void)
{
    xTaskCreate(bmi160_test, "bmi160_test", configMINIMAL_STACK_SIZE * 8, NULL, configMAX_PRIORITIES - 1, &xTaskHandlePtr);
}

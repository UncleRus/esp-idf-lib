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

/* Example for tap detection */

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 2.44Mhz

TaskHandle_t xTaskHandlePtr = NULL;

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


void bmi160_task(void *pvParameters)
{
    
    bmi160_t bmi160_dev;
    memset(&bmi160_dev.i2c_dev, 0, sizeof(i2c_dev_t));

    i2cdev_init();

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

    ESP_LOGI(TAG,"Example for tap detection\n");

    ESP_LOGI(TAG,"Initializing BMI160\n");
    ESP_ERROR_CHECK(bmi160_init(&bmi160_dev, BMI160_I2C_ADDRESS_VDD, I2C_PORT, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO));

    ESP_ERROR_CHECK(bmi160_self_test(&bmi160_dev));

    bmi160_conf_t bmi160_conf = {
        .accRange = BMI160_ACC_RANGE_2G,
        .accOdr = BMI160_ACC_ODR_100HZ,
        .accAvg = BMI160_ACC_LP_AVG_8,
        .accMode = BMI160_PMU_ACC_LOW_POWER,
        .gyrRange = BMI160_GYR_RANGE_125DPS,
        .gyrOdr = BMI160_GYR_ODR_100HZ,
        .gyrMode = BMI160_PMU_GYR_SUSPEND,
        .accUs = 1u
    };
    
    ESP_ERROR_CHECK(bmi160_start(&bmi160_dev, &bmi160_conf));

    //configure tap detection
    bmi160_tap_conf_t tapConf = {
        .tapQuiet = BMI160_TAP_QUIET_20MS,
        .tapShock = BMI160_TAP_SHOCK_75MS,
        .tapDur = BMI160_TAP_DUR_250MS,
        .tapTh = BMI160_TAP_TH_1G,
        .tapMode = BMI160_TAP_MODE_SINGLE
    };

    ESP_ERROR_CHECK(bmi160_enable_tap_detection(&bmi160_dev, &tapConf));


    //enable interrupt on bmi160
    bmi160_int_out_conf_t intOutConf = {
        .intPin = BMI160_PIN_INT1,
        .intEnable = BMI160_INT_ENABLE,
        .intOd = BMI160_INT_PUSH_PULL,
        .intLevel = BMI160_INT_ACTIVE_HIGH
    };
    bmi160_enable_int_tap(&bmi160_dev, &intOutConf);

    while(1)
    {
        //wait for interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //tap detected
        ESP_LOGI(TAG,"Tap detected\n");
    }

    bmi160_free(&bmi160_dev);
}


void app_main(void)
{
    xTaskCreate(bmi160_task, "bmi160_task", configMINIMAL_STACK_SIZE * 8, NULL, configMAX_PRIORITIES - 1, &xTaskHandlePtr);
}

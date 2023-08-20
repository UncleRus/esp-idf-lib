#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <tsl2591.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "tsl2591_interrupt_example";

static QueueHandle_t isr_evt_queue = NULL;


static void IRAM_ATTR isr_handler(void *arg)
{
    // Be careful to pass a address to a  pointer.
    xQueueSendFromISR(isr_evt_queue, &arg, NULL);
}

static void isr_task(void *arg)
{
    ESP_LOGI(TAG, "isr_task started.");
    tsl2591_t *dev = NULL;
    while (1)
    {
        ESP_LOGI(TAG, "Wait for interrupt.");
        if (xQueueReceive(isr_evt_queue, &dev, portMAX_DELAY))
        {
            bool als_intr = false;
            bool als_np_intr = false;
            ESP_ERROR_CHECK(tsl2591_get_als_intr_flag(dev, &als_intr));
            ESP_ERROR_CHECK(tsl2591_get_np_intr_flag(dev, &als_np_intr));

            if (als_intr & !als_np_intr)
            {
                printf("ALS interrupt\n");
                vTaskDelay(pdMS_TO_TICKS(2000));
                ESP_LOGI(TAG, "Clear ALS Interrupt flag.");
                ESP_ERROR_CHECK(tsl2591_clear_als_intr(dev));
            }
            else if (!als_intr & als_np_intr)
            {
                printf("ALS no persist interrupt\n");
                vTaskDelay(pdMS_TO_TICKS(4000));
                ESP_LOGI(TAG, "Clear ALS NP interrupt flag.");
                ESP_ERROR_CHECK(tsl2591_clear_als_np_intr(dev));
            }
            else
            {
                printf("ALS and ALS NP interrupt");
                vTaskDelay(pdMS_TO_TICKS(6000));
                ESP_LOGI(TAG, "Clear both interrupts.");
                ESP_ERROR_CHECK(tsl2591_clear_both_intr(dev));
            }
        }
    }
}

void tsl2591_task(void *pvParameters)
{
    ESP_LOGI(TAG, "tsl2591 task started.");
    tsl2591_t *dev = (tsl2591_t*)pvParameters;
    uint16_t channel0, channel1;
    float lux;

    assert(dev != NULL);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_ERROR_CHECK(tsl2591_get_channel_data(dev, &channel0, &channel1));
        ESP_ERROR_CHECK(tsl2591_calculate_lux(dev, channel0, channel1, &lux));
        printf("Channel 0: 0x%x\n", channel0);
        printf("Channel 1: 0x%x\n", channel1);
        printf("Lux: %f\n", lux);
        printf("GPIO %d: %s\n", CONFIG_EXAMPLE_INTR_GPIO, gpio_get_level(CONFIG_EXAMPLE_INTR_GPIO) ? "HIGH" : "LOW");
    }
}

void app_main()
{
    // i2cdev initialisation
    ESP_ERROR_CHECK(i2cdev_init());

    // tsl2591 initialisation
    tsl2591_t *dev = calloc(1, sizeof(tsl2591_t));

    ESP_ERROR_CHECK(tsl2591_init_desc(dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(tsl2591_init(dev));

    ESP_ERROR_CHECK(tsl2591_set_power_status(dev, TSL2591_POWER_OFF));
    ESP_ERROR_CHECK(tsl2591_set_als_status(dev, TSL2591_ALS_OFF));

    uint16_t low_thresh_als = 0x00;
    uint16_t high_thresh_als = 0x170;
    uint16_t low_thresh_als_np = 0x00;
    uint16_t high_thresh_als_np = 0x1000;

    ESP_LOGI(TAG, "Turn both interrupts on.");
    ESP_ERROR_CHECK(tsl2591_set_interrupt(dev, TSL2591_ALS_INTR_BOTH_ON));
    ESP_LOGI(TAG, "Set interrupt thresholds.");
    ESP_ERROR_CHECK(tsl2591_als_set_low_threshold(dev, low_thresh_als));
    ESP_ERROR_CHECK(tsl2591_als_set_high_threshold(dev, high_thresh_als));
    ESP_ERROR_CHECK(tsl2591_no_persist_set_low_threshold(dev, low_thresh_als_np));
    ESP_ERROR_CHECK(tsl2591_no_persist_set_high_threshold(dev, high_thresh_als_np));
    ESP_LOGI(TAG, "Set persistence filter.");
    ESP_ERROR_CHECK(tsl2591_set_persistence_filter(dev, TSL2591_60_CYCLES));
    ESP_LOGI(TAG, "Set sleep after interrupt.");
    ESP_ERROR_CHECK(tsl2591_set_sleep_after_intr(dev, TSL2591_SLEEP_AFTER_ON));

    ESP_LOGI(TAG, "Clear old interrupts.");
    ESP_ERROR_CHECK(tsl2591_clear_both_intr(dev));

    ESP_LOGI(TAG, "Start integration cycle.");
    ESP_ERROR_CHECK(tsl2591_set_power_status(dev, TSL2591_POWER_ON));
    ESP_ERROR_CHECK(tsl2591_set_als_status(dev, TSL2591_ALS_ON));
    vTaskDelay(pdMS_TO_TICKS(600));

    // Setup external interrupt on pin CONFIG_EXAMPLE_INTR_GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_INTR_GPIO);
    io_conf.pull_up_en = 1; // tsl2591 active low interrupt.
    io_conf.pull_down_en = 0;
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    isr_evt_queue = xQueueCreate(10, sizeof(tsl2591_t*));

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_EXAMPLE_INTR_GPIO, isr_handler, (void*)dev);

    ESP_LOGI(TAG, "Dispatch isr_task.");
    xTaskCreate(isr_task, "interrupt_task", 2048, NULL, 10, NULL);
    ESP_LOGI(TAG, "Dispatch tsl2591_task.");
    xTaskCreatePinnedToCore(tsl2591_task, "tsl2591_test", 8192, (void*)dev, 5, NULL, APP_CPU_NUM);
}


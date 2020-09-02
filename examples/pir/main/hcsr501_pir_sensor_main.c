#include <FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <pir.h>

#define RTOS_DELAY_1SEC (1000 / portTICK_PERIOD_MS)
#define RTOS_DELAY_60SEC (60 * 1000 / portTICK_PERIOD_MS)
#define RTOS_TASK_PRIORITY_NORMAL (5)
#define GPIO_LED_PIN_SEL (1ULL << CONFIG_MY_LED_ON_DEVBOARD_GPIO_NUM)

#if CONFIG_MY_LED_ON_DEVBOARD_WIRING_TYPE == 1
#define LED_ON_LEVEL 0
#define LED_OFF_LEVEL 1
#elif CONFIG_MY_LED_ON_DEVBOARD_WIRING_TYPE == 2
#define LED_ON_LEVEL 1
#define LED_OFF_LEVEL 0
#endif

/*
 * Logging
 */
static const char TAG[] = "myapp";

/*
 * KConfig: LED, WIFI
 */
static const int MY_LED_ON_DEVBOARD_GPIO_NUM = CONFIG_MY_LED_ON_DEVBOARD_GPIO_NUM;
static const int MY_LED_ON_DEVBOARD_WIRING_TYPE = CONFIG_MY_LED_ON_DEVBOARD_WIRING_TYPE;

static const int MY_SENSOR_DATA_GPIO_NUM = CONFIG_MY_SENSOR_DATA_GPIO_NUM;

/*
 * FreeRTOS settings
 */
#define MYAPP_RTOS_TASK_STACK_SIZE_8K (8192)
#define MYAPP_RTOS_TASK_PRIORITY_NORMAL (RTOS_TASK_PRIORITY_NORMAL)

/**
 * @brief Initialize LED GPIO
 */

static esp_err_t init_led_pin() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    return gpio_config(&io_conf);
}

static void timer_task(void *pvParameter) {
    while (1) {
        vTaskDelay(RTOS_DELAY_60SEC);
        ESP_LOGI(TAG, "60 sec passed from the startup. the sensor should be ready.");
        goto finish;
    }

finish:
    vTaskDelete(NULL);
}

/**
 * TASKS
 */
/**
 * @brief This task logs the status of the DATA pin every second.
 *
 * @important This uses a FreeRTOS Semaphore so the CPU is not being clogged when waiting for a new event.
 *
 */

void sensor_task(void *pvParameter) {
    ESP_LOGI(TAG, "%s()", __FUNCTION__);

    uint32_t total_nbr_of_detections = 0;

    /********************************************************************************
     * Reuseable variables
     */
    esp_err_t f_retval;

    /********************************************************************************
     * LED
     */
    ESP_LOGI(TAG, "\n\n***SECTION: LED***");
    ESP_LOGI(TAG, "  MY_LED_ON_DEVBOARD_GPIO_NUM:    %i", MY_LED_ON_DEVBOARD_GPIO_NUM);
    ESP_LOGI(TAG, "  MY_LED_ON_DEVBOARD_WIRING_TYPE: %i", MY_LED_ON_DEVBOARD_WIRING_TYPE);
    f_retval = init_led_pin();
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "init_led_pin() err %i %s", f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    /********************************************************************************
     * SENSOR
     *
     */
    ESP_LOGI(TAG, "HC-SR051 PIR SENSOR");
    ESP_LOGW(TAG, "  @doc The on-board LED will blink when human movement is detected by the sensor.");
    ESP_LOGW(TAG,
            "  @doc The pin stays HIGH for at least 3 seconds when a movement is detected (depends on the Time Decay potmeter setting)");

    ESP_LOGW(TAG, "  @important After power-on, WAIT 60 seconds so the PIR Sensor can calibrate itself!");
    ESP_LOGW(TAG, "  @important     + Keep the area clear of human movement.");
    ESP_LOGW(TAG, "  @important     + False positives might occur during calibration.");

    // Init sensor
    pir_config_t hcsr501_config = MJD_HCSR501_CONFIG_DEFAULT();
    hcsr501_config.data_gpio_num = MY_SENSOR_DATA_GPIO_NUM;

    f_retval = pir_init(&hcsr501_config);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "pir_init() err %i %s", f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    while (true) {
        if (xSemaphoreTake(hcsr501_config.isr_semaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Task sensor_task: ***SENSOR: MOTION DETECTED*** (total_nbr_of_detections %i)",
                    ++total_nbr_of_detections);
        }
    }

    // LABEL
    cleanup: ;

    /********************************************************************************
     * Task Delete
     * @doc Passing NULL will end the current task
     */
    vTaskDelete(NULL);
}

/**
 * @brief This task logs the status of the DATA pin every second.
 *
 */
void other_task(void *pvParameter) {
    int level;
    ESP_LOGI(TAG, "%s()", __FUNCTION__);

    while (true) {
        level = gpio_get_level(MY_SENSOR_DATA_GPIO_NUM);
        ESP_LOGI(TAG, "Task other_task: every 1 second - INPUT GPIO#%i: Actual value=%i",
                MY_SENSOR_DATA_GPIO_NUM,
                level);
        gpio_set_level(CONFIG_MY_LED_ON_DEVBOARD_GPIO_NUM, level == 1 ? LED_ON_LEVEL : LED_OFF_LEVEL);
        vTaskDelay(RTOS_DELAY_1SEC);
    }
}

/*
 * MAIN
 */
void app_main() {
    ESP_LOGD(TAG, "%s()", __FUNCTION__);

    BaseType_t xReturned;

    /********************************************************************************
     * MY STANDARD Init
     */
    ESP_LOGI(TAG, "@doc Wait X seconds after power-on (start logic analyzer, let sensors become active!)");
    vTaskDelay(RTOS_DELAY_1SEC);

    /**********
     * TASK: sensor @ core APP_CPU_NUM (1)
     *
     * @important For stability (RMT + Wifi + BT): always use xTaskCreatePinnedToCore(APP_CPU_NUM) [Opposed to xTaskCreate()]
     *
     */
    xReturned = xTaskCreatePinnedToCore(&sensor_task,
                                        "sensor_task (name)",
                                        MYAPP_RTOS_TASK_STACK_SIZE_8K,
                                        NULL,
                                        MYAPP_RTOS_TASK_PRIORITY_NORMAL,
                                        NULL,
                                        APP_CPU_NUM);
    if (xReturned == pdPASS) {
        ESP_LOGI(TAG, "OK Task sensor_task has been created, and is running right now");
    }
    ESP_LOGI(TAG, "@doc Wait 1 second before starting another task (first task configures the GPIO#)");
    vTaskDelay(RTOS_DELAY_1SEC);

    /**********
     * TASK something else @ core APP_CPU_NUM (1)
     *
     * @important For stability (RMT + Wifi + BT): always use xTaskCreatePinnedToCore(APP_CPU_NUM) [Opposed to xTaskCreate()]
     */
    xReturned = xTaskCreatePinnedToCore(&other_task,
                                        "othertask (name)",
                                        MYAPP_RTOS_TASK_STACK_SIZE_8K,
                                        NULL,
                                        MYAPP_RTOS_TASK_PRIORITY_NORMAL,
                                        NULL,
                                        APP_CPU_NUM);
    if (xReturned == pdPASS) {
        ESP_LOGI(TAG, "OK Task other_task has been created, and is running right now");
    }
    /*
     * Create a task that simply tells 60 sec passed.
     */
    xReturned = xTaskCreatePinnedToCore(&timer_task,
                                        "othertask (name)",
                                        MYAPP_RTOS_TASK_STACK_SIZE_8K,
                                        NULL,
                                        MYAPP_RTOS_TASK_PRIORITY_NORMAL,
                                        NULL,
                                        APP_CPU_NUM);
    if (xReturned == pdPASS) {
        ESP_LOGI(TAG, "OK Task timer_task has been created, and is running right now");
    }

    /**********
     * END
     */
    ESP_LOGI(TAG, "END %s()", __FUNCTION__);
}

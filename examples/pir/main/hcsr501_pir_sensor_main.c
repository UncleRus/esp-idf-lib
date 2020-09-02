#include "mjd.h"
#include "mjd_hcsr501.h"

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

/*
 * GPIOMUX sensor data pin -> on-board LED
 *
 * Use the MUX to route the GPIO input pin receiving the signal from the sensor to the GPIO Output pin of the on-board LED.
 *
 * Only the loopback signals 224-228 can be configured to be routed from one input GPIO directly to another output GPIO.
 *
 * KConfig wiring types:
 *   + Adafruit Huzzah32: 1 MY_LED_ON_DEVBOARD_WIRING_TYPE_LED_LOW_SIDE
 *   + LOLIN D32:         2 MY_LED_ON_DEVBOARD_WIRING_TYPE_LED_HIGH_SIDE
 *
 */
void gpio_setup_gpiomux_sensor_to_led() {
    ESP_LOGD(TAG, "%s()", __FUNCTION__);

    bool invert_signal = MY_LED_ON_DEVBOARD_WIRING_TYPE == 2;
    gpio_matrix_in(MY_SENSOR_DATA_GPIO_NUM, SIG_IN_FUNC224_IDX, invert_signal);
    gpio_matrix_out(MY_LED_ON_DEVBOARD_GPIO_NUM, SIG_IN_FUNC224_IDX, false, false);
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

    mjd_log_memory_statistics();

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

    mjd_log_memory_statistics();

    mjd_led_config_t led_config =
        { 0 };
    led_config.gpio_num = MY_LED_ON_DEVBOARD_GPIO_NUM;
    led_config.wiring_type = MY_LED_ON_DEVBOARD_WIRING_TYPE; // 1 GND MCU Huzzah32 | 2 VCC MCU Lolin32lite
    mjd_led_config(&led_config);

    gpio_setup_gpiomux_sensor_to_led();

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
    mjd_hcsr501_config_t hcsr501_config = MJD_HCSR501_CONFIG_DEFAULT();
    hcsr501_config.data_gpio_num = MY_SENSOR_DATA_GPIO_NUM;

    f_retval = mjd_hcsr501_init(&hcsr501_config);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "mjd_hcsr501_init() err %i %s", f_retval, esp_err_to_name(f_retval));
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
    ESP_LOGI(TAG, "%s()", __FUNCTION__);

    mjd_log_memory_statistics();

    while (true) {
        ESP_LOGI(TAG, "Task other_task: every 1 second - INPUT GPIO#%i: Actual value=%i", MY_SENSOR_DATA_GPIO_NUM,
                gpio_get_level(MY_SENSOR_DATA_GPIO_NUM));
        vTaskDelay(RTOS_DELAY_1SEC);
    }
}

/*
 * MAIN
 */
void app_main() {
    ESP_LOGD(TAG, "%s()", __FUNCTION__);

    BaseType_t xReturned;

    mjd_log_memory_statistics();

    /********************************************************************************
     * MY STANDARD Init
     */
    mjd_log_wakeup_details();
    mjd_log_chip_info();
    mjd_log_time();
    mjd_log_memory_statistics();
    ESP_LOGI(TAG, "@doc Wait X seconds after power-on (start logic analyzer, let sensors become active!)");
    vTaskDelay(RTOS_DELAY_1SEC);

    /**********
     * TASK: sensor @ core APP_CPU_NUM (1)
     *
     * @important For stability (RMT + Wifi + BT): always use xTaskCreatePinnedToCore(APP_CPU_NUM) [Opposed to xTaskCreate()]
     *
     */
    xReturned = xTaskCreatePinnedToCore(&sensor_task, "sensor_task (name)", MYAPP_RTOS_TASK_STACK_SIZE_8K, NULL,
    MYAPP_RTOS_TASK_PRIORITY_NORMAL, NULL,
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
    xReturned = xTaskCreatePinnedToCore(&other_task, "othertask (name)", MYAPP_RTOS_TASK_STACK_SIZE_8K, NULL,
    MYAPP_RTOS_TASK_PRIORITY_NORMAL, NULL,
    APP_CPU_NUM);
    if (xReturned == pdPASS) {
        ESP_LOGI(TAG, "OK Task other_task has been created, and is running right now");
    }

    /**********
     * END
     */
    ESP_LOGI(TAG, "END %s()", __FUNCTION__);
}

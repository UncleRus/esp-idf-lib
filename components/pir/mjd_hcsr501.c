/*
 * HC-SR501 PIR Human Infrared Sensor Module Including Lens
 *
 */

// Component header file(s)
#include "mjd.h"
#include "mjd_hcsr501.h"

/*
 * Logging
 */
static const char TAG[] = "mjd_hcsr501";

/*
 * INTERRUPT
 */
static SemaphoreHandle_t _hcsr501_config_isr_semaphore;

void sensor_gpio_isr_handler(void* arg) {
    // @param pxHigherPriorityTaskWoken to pdTRUE if giving the semaphore caused a task to unblock, and the unblocked task has a priority higher than the currently running task.
    //        If xSemaphoreGiveFromISR() sets this value to pdTRUE then a context switch should be requested before the interrupt is exited.
    //        portYIELD_FROM_ISR() wakes up the imu_task immediately (instead of on next FreeRTOS tick).
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_hcsr501_config_isr_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/*********************************************************************************
 * PUBLIC.
 *
 *
 */
esp_err_t mjd_hcsr501_init(mjd_hcsr501_config_t* param_ptr_config) {
    ESP_LOGD(TAG, "%s()", __FUNCTION__);

    esp_err_t f_retval = ESP_OK;

    if (param_ptr_config->is_init == true) {
        f_retval = ESP_FAIL;
        ESP_LOGE(TAG, "ABORT. mjd_hcsr501_init() component is already init'd | err %i %s", f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    // WAIT 5 seconds after power-on so the PIR Sensor can calibrate itself!
    //      @important     + Keep the area clear of human movement.
    //      @important      + False positives might occur during calibration.
    vTaskDelay(RTOS_DELAY_5SEC);

    // Semaphore
    // @important Clone the semaphore locally (so the ISR can reference the semaphore handle).
    // @doc Binary semaphores created using xSemaphoreCreateBinary() are created in a state such that the semaphore must first be 'given' before it can be 'taken'!
    // @doc The required RAM is automatically allocated from the FreeRTOS heap (opposed to from the local stack).
    param_ptr_config->isr_semaphore = xSemaphoreCreateBinary();
    if (param_ptr_config->isr_semaphore == NULL) {
        f_retval = ESP_FAIL;
        ESP_LOGE(TAG, "ABORT. xSemaphoreCreateBinary() failed | err %i %s", f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }
    _hcsr501_config_isr_semaphore = param_ptr_config->isr_semaphore;

    // GPIO
    //   @doc GPIO_INTR_POSEDGE = GPIO interrupt type: rising edge (LOW->HIGH)
    gpio_config_t io_conf =
        { 0 };
    io_conf.pin_bit_mask = (1ULL << param_ptr_config->data_gpio_num);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // @important HC-SR501 data pin
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE; // @validvalues GPIO_INTR_DISABLE GPIO_INTR_ANYEDGE GPIO_INTR_NEGEDGE GPIO_INTR_POSEDGE
    f_retval = gpio_config(&io_conf);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "ABORT. gpio_config() failed | err %i %s", f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    // ISR
    // @doc ESP_INTR_FLAG_LEVEL1 Accept a Level 1 interrupt vector (lowest priority)
    f_retval = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "ABORT. gpio_install_isr_service() failed | err %i %s", f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }
    f_retval = gpio_isr_handler_add(param_ptr_config->data_gpio_num, sensor_gpio_isr_handler, NULL);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "ABORT. gpio_isr_handler_add() failed | err %i %s", f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    // Mark OK
    param_ptr_config->is_init = true;

    // LABEL
    cleanup: ;

    return f_retval;
}

esp_err_t mjd_hcsr501_deinit(mjd_hcsr501_config_t* param_ptr_config) {
    ESP_LOGD(TAG, "%s()", __FUNCTION__);

    esp_err_t f_retval = ESP_OK;

    if (param_ptr_config->is_init == false) {
        f_retval = ESP_FAIL;
        ESP_LOGE(TAG, "ABORT. mjd_hcsr501_deinit() component was not init'd | err %i %s", f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    // ISR
    f_retval = gpio_isr_handler_remove(param_ptr_config->data_gpio_num);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "ABORT. gpio_isr_handler_remove() failed | err %i %s", f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }
    gpio_uninstall_isr_service(); // @returns void

    // SEMAPHORE
    vSemaphoreDelete(param_ptr_config->isr_semaphore);
    param_ptr_config->isr_semaphore = NULL;
    _hcsr501_config_isr_semaphore = param_ptr_config->isr_semaphore;

    // Mark OK
    param_ptr_config->is_init = false;

    // LABEL
    cleanup: ;

    return f_retval;
}

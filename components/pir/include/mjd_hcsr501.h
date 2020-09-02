/*
 *
 */
#ifndef __MJD_HCSR501_H__
#define __MJD_HCSR501_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Data structs
 *
 * @doc https://www.geeksforgeeks.org/mutex-vs-semaphore/
 *
 */
typedef struct {
        bool is_init;
        gpio_num_t data_gpio_num;
        SemaphoreHandle_t isr_semaphore;
} mjd_hcsr501_config_t;

#define MJD_HCSR501_CONFIG_DEFAULT() { \
    .is_init = false, \
    .data_gpio_num = GPIO_NUM_MAX, \
    .isr_semaphore = NULL \
}

/**
 * Function declarations
 */
esp_err_t mjd_hcsr501_init(mjd_hcsr501_config_t* ptr_param_config);
esp_err_t mjd_hcsr501_deinit(mjd_hcsr501_config_t* ptr_param_config);

#ifdef __cplusplus
}
#endif

#endif /* __MJD_HCSR501_H__ */

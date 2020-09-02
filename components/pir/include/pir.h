/*
 * MIT License
 *
 * Copyright (c) 2019 Nocluna
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __MJD_HCSR501_H__
#define __MJD_HCSR501_H__

#include <FreeRTOS.h>
#include <driver/gpio.h>
#include <freertos/semphr.h>

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
} pir_config_t;

#define MJD_HCSR501_CONFIG_DEFAULT() { \
    .is_init = false, \
    .data_gpio_num = GPIO_NUM_MAX, \
    .isr_semaphore = NULL \
}

/**
 * Function declarations
 */
esp_err_t pir_init(pir_config_t* ptr_param_config);
esp_err_t pir_deinit(pir_config_t* ptr_param_config);

#ifdef __cplusplus
}
#endif

#endif /* __MJD_HCSR501_H__ */

/*
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file ultrasonic.h
 * @defgroup ultrasonic ultrasonic
 * @{
 *
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and so on
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ERR_ULTRASONIC_PING         0x200
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202

/**
 * Device descriptor
 */
typedef struct
{
    gpio_num_t trigger_pin; //!< GPIO output pin for trigger
    gpio_num_t echo_pin;    //!< GPIO input pin for echo
} ultrasonic_sensor_t;

/**
 * @brief Init ranging module
 *
 * @param dev Pointer to the device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev);

/**
 * @brief Measure time between ping and echo
 *
 * @param dev Pointer to the device descriptor
 * @param max_time_us Maximal time to wait for echo
 * @param[out] time_us Time, us
 * @return `ESP_OK` on success, otherwise:
 *         - ::ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
 *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
 *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
 */
esp_err_t ultrasonic_measure_raw(const ultrasonic_sensor_t *dev, uint32_t max_time_us, uint32_t *time_us);

/**
 * @brief Measure distance in meters
 *
 * @param dev Pointer to the device descriptor
 * @param max_distance Maximal distance to measure, meters
 * @param[out] distance Distance in meters
 * @return `ESP_OK` on success, otherwise:
 *         - ::ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
 *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
 *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
 */
esp_err_t ultrasonic_measure(const ultrasonic_sensor_t *dev, float max_distance, float *distance);

/**
 * @brief Measure distance in centimeters
 *
 * @param dev Pointer to the device descriptor
 * @param max_distance Maximal distance to measure, centimeters
 * @param[out] distance Distance in centimeters
 * @return `ESP_OK` on success, otherwise:
 *         - ::ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
 *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
 *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
 */
esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t *dev, uint32_t max_distance, uint32_t *distance);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ULTRASONIC_H__ */

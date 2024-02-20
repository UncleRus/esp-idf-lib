/*
 * Copyright (c) 2024 Jakub Turek <qb4.dev@gmail.com>
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
 * @file impulse_sensor.h
 * @defgroup impulse_sensor impulse_sensor
 * @{
 *
 * ESP-IDF driver for impulse sensors
 *
 * Copyright (c) 2024 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __IMPULSE_SENSOR_H__
#define __IMPULSE_SENSOR_H__

#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMP_SENSOR_DEFAULT_SF          1.0  ///< default scale factor
#define IMP_SENSOR_DEFAULT_MEAS_PERIOD 1000 ///< default measurement period[1sec]

/**
 * Device descriptor
 */

typedef void *imp_sensor_t;

typedef struct
{
    gpio_num_t input_pin;       //!< GPIO input pin
    const float scale_factor;   //!< scale factor
    const uint32_t meas_period; //!< measurement period[msecs]
} imp_sensor_config_t;

/**
 * @brief Init impulse sensor
 *
 * @param conf Pointer to sensor config
 * @param[out] imp_sensor Pointer to created sensor object
 * @return `ESP_OK` on success
 */
esp_err_t imp_sensor_init(const imp_sensor_config_t *conf, imp_sensor_t *imp_sensor);

/**
 * @brief Deinit impulse sensor
 *
 * @param imp_sensor Pointer to sensor device
 * @return `ESP_OK` on success
 */
esp_err_t imp_sensor_deinit(imp_sensor_t *imp_sensor);

/**
 * @brief Deinit impulse sensor
 *
 * @param imp_sensor Pointer to sensor device
 * @param[out] value Output value multiplied by scale factor
 * @return `ESP_OK` on success
 */
esp_err_t imp_sensor_get_value(imp_sensor_t *imp_sensor, float *value);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __IMPULSE_SENSOR_H__ */

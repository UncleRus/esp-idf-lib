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
 * @file anemometer.h
 * @defgroup misc wind
 * @{
 *
 * ESP-IDF driver for impulse wind speed sensors(anemometers)
 *
 * Copyright (c) 2024 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __ANEMOMETER_H__
#define __ANEMOMETER_H__

#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANEMOMETER_DEFAULT_SF (1.75/20) //!< 1.75 m/s = 20 pps

/**
 * Device descriptor
 */

typedef void *anemometer_t;

typedef struct
{
    gpio_num_t input_pin;      //!< GPIO input pin
    const float scale_factor;  //!< scale factor
} anemometer_config_t;

/**
 * @brief Init anemometer sensor
 *
 * @param config Pointer to the device config
 * @return anemometer device or NULL if failed
 */
anemometer_t anemometer_init(const anemometer_config_t *config);

/**
 * @brief Deinit anemometer sensor
 *
 * @param anemometer Pointer to the anemometer device
 * @return `ESP_OK` on success
 */
esp_err_t anemometer_deinit(anemometer_t *anemometer);

/**
 * @brief Deinit anemometer sensor
 *
 * @param anemometer Pointer to the anemometer device
 * @param[out] speed calculated wind speed in [m/s]
 * @return `ESP_OK` on success
 */
esp_err_t anemometer_get_wind_speed(anemometer_t *anemometer, float *speed);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ANEMOMETER_H__ */

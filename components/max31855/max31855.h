/*
 * Copyright (c) 2022 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file max31855.h
 * @defgroup max31855 max31855
 * @{
 *
 * ESP-IDF driver for MAX31855 cold-junction compensated
 * thermocouple-to-digital converter
 *
 * Copyright (c) 2022 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MAX31855_H__
#define __MAX31855_H__

#include <stdint.h>
#include <stdbool.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX31855_MAX_CLOCK_SPEED_HZ (5000000) // 5 MHz

/**
 * Device descriptor
 */
typedef struct
{
    spi_device_interface_config_t spi_cfg;  /**< SPI device configuration */
    spi_device_handle_t spi_dev;            /**< SPI device handler */
} max31855_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev            Device descriptor
 * @param host           SPI host
 * @param cs_pin         CS GPIO number
 * @param clock_speed_hz SPI clock speed, Hz
 * @return `ESP_OK` on success
 */
esp_err_t max31855_init_desc(max31855_t *dev, spi_host_device_t host, uint32_t clock_speed_hz, gpio_num_t cs_pin);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max31855_free_desc(max31855_t *dev);

/**
 * @brief Read temperatures and sensor status
 *
 * @param dev       Device descriptor
 * @param[out] tc_t Thermocouple temperature, degrees Celsius
 * @param[out] cj_t Cold junction temperature, degrees Celsius (NULL-able)
 * @param[out] scv  true when the thermocouple is short-circuited to VCC
 * @param[out] scg  true when the thermocouple is short-circuited to GND
 * @param[out] oc   true when the thermocouple is open (no connections)
 * @return
 */
esp_err_t max31855_get_temperature(max31855_t *dev, float *tc_t, float *cj_t, bool *scv, bool *scg, bool *oc);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MAX31855_H__ */

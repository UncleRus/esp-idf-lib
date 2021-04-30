/*
 * Copyright (c) 2016 Bernhard Guillon <Bernhard.Guillon@begu.org>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file ms5611.h
 * @defgroup ms5611 ms5611
 * @{
 *
 * ESP-IDF driver for barometric pressure sensor MS5611-01BA03
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Bernhard Guillon <Bernhard.Guillon@begu.org>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MS5611_H__
#define __MS5611_H__

#include <stdint.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MS5611_ADDR_CSB_HIGH    0x76
#define MS5611_ADDR_CSB_LOW     0x77

/**
 * Oversampling ratio
 */
typedef enum
{
    MS5611_OSR_256  = 0x00, //!< 256 samples per measurement
    MS5611_OSR_512  = 0x02, //!< 512 samples per measurement
    MS5611_OSR_1024 = 0x04, //!< 1024 samples per measurement
    MS5611_OSR_2048 = 0x06, //!< 2048 samples per measurement
    MS5611_OSR_4096 = 0x08  //!< 4096 samples per measurement
} ms5611_osr_t;

/**
 * Configuration data
 */
typedef struct
{
    uint16_t sens;       //!< C1 Pressure sensitivity                             | SENS_t1
    uint16_t off;        //!< C2 Pressure offset                                  | OFF_t1
    uint16_t tcs;        //!< C3 Temperature coefficient of pressure sensitivity  | TCS
    uint16_t tco;        //!< C4 Temperature coefficient of pressure offset       | TCO
    uint16_t t_ref;      //!< C5 Reference temperature                            | T_ref
    uint16_t tempsens;   //!< C6 Temperature coefficient of the temperature       | TEMPSENSE
} ms5611_config_data_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;                //!< I2C device settings
    ms5611_osr_t osr;                 //!< Oversampling setting
    ms5611_config_data_t config_data; //!< Device configuration, filled upon initialize
} ms5611_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr I2C address, `MS5611_ADDR_CSB_HIGH` or `MS5611_ADDR_CSB_LOW`
 * @param port I2C port
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_init_desc(ms5611_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_free_desc(ms5611_t *dev);

/**
 * @brief Init MS5611-01BA03
 *
 * Reset device and read calibration data
 *
 * @param dev Device descriptor
 * @param osr Oversampling ratio
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_init(ms5611_t *dev, ms5611_osr_t osr);

/**
 * @brief Measure pressure and temperature
 *
 * @param dev Device descriptor
 * @param[out] pressure Pressure, Pa
 * @param[out] temperature Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_get_sensor_data(ms5611_t *dev, int32_t *pressure, float *temperature);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MS5611_H__ */

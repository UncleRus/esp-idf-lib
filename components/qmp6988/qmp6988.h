/*
 * Copyright (c) 2019 Ruslan V. Uss (https://github.com/UncleRus)
 * Copyright (c) 2022 m5stack (https://github.com/m5stack)
 * Copyright (c) 2024 vonguced (https://github.com/vonguced)
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
 * @file qmp6988.h
 * @defgroup qmp6988 qmp6988
 * @{
 *
 * ESP-IDF driver for QMP6988 digital temperature and pressure sensor
 *
 * Code based on m5stack <https://github.com/m5stack/M5Unit-ENV>\n
 * and Ruslan V. Uss <https://github.com/UncleRus>
 *
 * Copyright (c) 2019 Ruslan V. Uss (https://github.com/UncleRus)\n
 * Copyright (c) 2022 m5stack (https://github.com/m5stack)\n
 * Copyright (c) 2024 vonguced (https://github.com/vonguced)\n
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __QMP6988_H__
#define __QMP6988_H__

#include <stdbool.h>
#include <stdint.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QMP6988_I2C_ADDR_GND 0x70
#define QMP6988_I2C_ADDR_VDD 0x56

/**
 * Possible measurement modes
 */
typedef enum {
    QMP6988_SLEEP_MODE = 0x00,  //!< sleep mode
    QMP6988_FORCED_MODE = 0x01, //!< one measurement then sleep again
    QMP6988_NORMAL_MODE = 0x03  //!< power mode
} qmp6988_power_mode_t;

/**
 * Possible filter modes
 */
typedef enum {
    QMP6988_FILTERCOEFF_OFF = 0x00,
    QMP6988_FILTERCOEFF_2 = 0x01,
    QMP6988_FILTERCOEFF_4 = 0x02,
    QMP6988_FILTERCOEFF_8 = 0x03,
    QMP6988_FILTERCOEFF_16 = 0x04,
    QMP6988_FILTERCOEFF_32 = 0x05
} qmp6988_filter_t;

/**
 * Possible oversampling modes
 */
typedef enum {
    QMP6988_OVERSAMPLING_SKIPPED = 0x00,
    QMP6988_OVERSAMPLING_1X = 0x01,
    QMP6988_OVERSAMPLING_2X = 0x02,
    QMP6988_OVERSAMPLING_4X = 0x03,
    QMP6988_OVERSAMPLING_8X = 0x04,
    QMP6988_OVERSAMPLING_16X = 0x05,
    QMP6988_OVERSAMPLING_32X = 0x06,
    QMP6988_OVERSAMPLING_64X = 0x07
} qmp6988_oversampling_t;

/**
 * Structure holding calibration data for QMP6988.
 */
typedef struct
{
    int32_t a0, b00;
    int32_t a1, a2;
    int64_t bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_ik_data_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev; //!< I2C device descriptor

    qmp6988_power_mode_t power_mode;            //!< used power mode
    qmp6988_filter_t filter_mode;               //!< used filter mode
    qmp6988_oversampling_t oversampling_t_mode; //!< used oversampling temp mode
    qmp6988_oversampling_t oversampling_p_mode; //!< used oversampling pressure mode

    qmp6988_ik_data_t ik;                       //!< used calibration data

    float temperature;                          //!< measured temperature
    float pressure;                             //!< measured pressure
} qmp6988_t;

/**
 * @brief Initialize device descriptor.
 *
 * @param dev       Device descriptor.
 * @param addr      Device address.
 * @param port      I2C port.
 * @param sda_gpio  SDA GPIO.
 * @param scl_gpio  SCL GPIO.
 * @return          `ESP_OK` on success.
 */
esp_err_t qmp6988_init_desc(qmp6988_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t qmp6988_free_desc(qmp6988_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t qmp6988_init(qmp6988_t *dev);

/**
 * @brief Set up power mode for QMP6988.
 *
 * @param dev           Device descriptor.
 * @param power_mode    Power mode to be set.
 * @return              `ESP_OK` on success.
 */
esp_err_t qmp6988_setup_powermode(qmp6988_t *dev, qmp6988_power_mode_t power_mode);

/**
 * @brief Set up filter mode for QMP6988.
 *
 * @param dev           Device descriptor.
 * @param filter_mode   Filter mode to be set.
 * @return              `ESP_OK` on success.
 */
esp_err_t qmp6988_set_filter(qmp6988_t *dev, qmp6988_filter_t filter_mode);

/**
 * @brief Set up pressure oversampling mode for QMP6988.
 *
 * @param dev                   Device descriptor.
 * @param oversampling_p_mode   Oversampling mode for pressure to be set.
 * @return                      `ESP_OK` on success.
 */
esp_err_t qmp6988_set_p_oversampling(qmp6988_t *dev, qmp6988_oversampling_t oversampling_p_mode);

/**
 * @brief Set up temperature oversampling mode for QMP6988.
 *
 * @param dev                   Device descriptor.
 * @param oversampling_t_mode   Oversampling mode for temperature to be set.
 * @return                      `ESP_OK` on success.
 */
esp_err_t qmp6988_set_t_oversampling(qmp6988_t *dev, qmp6988_oversampling_t oversampling_t_mode);

/**
 * @brief Calculate pressure based on QMP6988 sensor data.
 *
 * @param dev       Device descriptor.
 * @param[out] p    Calculated pressure in Pascals (Pa).
 * @return          `ESP_OK` on success.
 */
esp_err_t qmp6988_calc_pressure(qmp6988_t *dev, float *p);

/**
 * @brief Calculate temperature based on QMP6988 sensor data.
 *
 * @param dev       Device descriptor.
 * @param[out] t    Calculated temperature in degrees Celsius (°C).
 * @return          `ESP_OK` on success.
 */
esp_err_t qmp6988_calc_temperature(qmp6988_t *dev, float *t);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __QMP6988_H__ */

/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file max31725.h
 * @defgroup max31725 max31725
 * @{
 *
 * ESP-IDF driver for MAX31725/MAX31726 temperature sensors
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MAX31725_H__
#define __MAX31725_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#define MAX31725_I2C_ADDR_BASE 0x40 //!< See full list in datasheet

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Temperature data range
 */
typedef enum {
    MAX31725_FMT_NORMAL = 0, //!< -128 deg.C .. +127.99609375 deg.C (default)
    MAX31725_FMT_EXTENDED    //!< -64 deg.C .. +191.99609375 deg.C (max 150 deg.C)
} max31725_data_format_t;

/**
 * Fault queue size
 */
typedef enum {
    MAX31725_FAULTS_1 = 0, //!< 1 fault to trigger OS (default)
    MAX31725_FAULTS_2,     //!< 2 fault to trigger OS
    MAX31725_FAULTS_4,     //!< 4 fault to trigger OS
    MAX31725_FAULTS_6      //!< 6 fault to trigger OS
} max31725_fault_queue_t;

/**
 * OS polarity
 */
typedef enum {
    MAX31725_OS_LOW = 0, //!< OS active low (default)
    MAX31725_OS_HIGH     //!< OS active high
} max31725_os_polarity_t;

/**
 * Mode of OS operation
 */
typedef enum {
    MAX31725_OS_COMPARATOR = 0, //!< OS comparator mode (default)
    MAX31725_OS_INTERRUPT       //!< OS interrupt mode
} max31725_os_mode_t;

/**
 * Device operating mode
 */
typedef enum {
    MAX31725_MODE_CONTINUOUS = 0, //!< Continuous measurement mode (default)
    MAX31725_MODE_SHUTDOWN        //!< Shutdown mode
} max31725_mode_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port number
 * @param addr I2C address
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t max31725_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max31725_free_desc(i2c_dev_t *dev);

/**
 * @brief Read current device config
 *
 * @param dev Device descriptor
 * @param[out] mode Operating mode
 * @param[out] fmt Data format
 * @param[out] fq Fault queue size
 * @param[out] op OS polarity
 * @param[out] om OS mode
 * @return `ESP_OK` on success
 */
esp_err_t max31725_get_config(i2c_dev_t *dev, max31725_mode_t *mode, max31725_data_format_t *fmt, max31725_fault_queue_t *fq,
        max31725_os_polarity_t *op, max31725_os_mode_t *om);

/**
 * @brief Configure device
 *
 * @param dev Device descriptor
 * @param mode Operating mode
 * @param fmt Data format
 * @param fq Fault queue size
 * @param op OS polarity
 * @param om OS mode
 * @return `ESP_OK` on success
 */
esp_err_t max31725_set_config(i2c_dev_t *dev, max31725_mode_t mode, max31725_data_format_t fmt, max31725_fault_queue_t fq,
        max31725_os_polarity_t op, max31725_os_mode_t om);

/**
 * @brief Made a single-shot measurement
 *
 * Works only when device is in shutdown mode.
 * Measurement time is ~50 ms.
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, deg.C
 * @param fmt Data format
 * @return `ESP_OK` on success
 */
esp_err_t max31725_one_shot(i2c_dev_t *dev, float *temp, max31725_data_format_t fmt);

/**
 * @brief Read temperature register
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, deg.C
 * @param fmt Data format
 * @return `ESP_OK` on success
 */
esp_err_t max31725_get_temperature(i2c_dev_t *dev, float *temp, max31725_data_format_t fmt);

/**
 * @brief Read OS threshold temperature
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, deg.C
 * @param fmt Data format
 * @return `ESP_OK` on success
 */
esp_err_t max31725_get_os_temp(i2c_dev_t *dev, float *temp, max31725_data_format_t fmt);

/**
 * @brief Set OS threshold temperature
 *
 * @param dev Device descriptor
 * @param temp Temperature, deg.C
 * @param fmt Data format
 * @return `ESP_OK` on success
 */
esp_err_t max31725_set_os_temp(i2c_dev_t *dev, float temp, max31725_data_format_t fmt);

/**
 * @brief Read OS hysteresis temperature
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, deg.C
 * @param fmt Data format
 * @return `ESP_OK` on success
 */
esp_err_t max31725_get_hysteresis_temp(i2c_dev_t *dev, float *temp, max31725_data_format_t fmt);

/**
 * @brief Set OS hysteresis temperature
 *
 * @param dev Device descriptor
 * @param temp Temperature, deg.C
 * @param fmt Data format
 * @return `ESP_OK` on success
 */
esp_err_t max31725_set_hysteresis_temp(i2c_dev_t *dev, float temp, max31725_data_format_t fmt);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MAX31725_H__ */

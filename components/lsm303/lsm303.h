/*
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
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
 * @file lsm303.h
 * @defgroup lsm303 lsm303
 * @{
 *
 * ESP-IDF Driver for LSM303: 3-axis accelerometer and magnetometer sensors
 *
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __LSM303_H__
#define __LSM303_H__

#include <stdint.h>
#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Default I2C address
 */

#define LSM303_ADDR_ACC 0x19
#define LSM303_ADDR_MAG 0x1E

/**
 * Accelerometer modes
 */
typedef enum {
    LSM303_ACC_MODE_NORMAL,          //!< Normal measurement mode; 10-bit
    LSM303_ACC_MODE_HIGH_RESOLUTION, //!< High resolution mode; 12-bit
    LSM303_ACC_MODE_LOW_POWER,       //!< Low power mode; 8-bit
} lsm303_acc_mode_t;

/**
 * Accelerometer data rates
 */
typedef enum {
    LSM303_ODR_POWER_DOWN = 0b0000, //!< Power-down mode
    LSM303_ODR_1_HZ = 0b0001,       //!< Normal / low-power mode (1 Hz)
    LSM303_ODR_10_HZ = 0b0010,      //!< Normal / low-power mode (10 Hz)
    LSM303_ODR_25_HZ = 0b0011,      //!< Normal / low-power mode (25 Hz)
    LSM303_ODR_50_HZ = 0b0100,      //!< Normal / low-power mode (50 Hz)
    LSM303_ODR_100_HZ = 0b0101,     //!< Normal / low-power mode (100 Hz)
    LSM303_ODR_200_HZ = 0b0110,     //!< Normal / low-power mode (200 Hz)
    LSM303_ODR_400_HZ = 0b0111,     //!< Normal / low-power mode (400 Hz)
    LSM303_ODR_1620_HZ = 0b1000,    //!< Low-power mode (1.620 kHz)
    LSM303_ODR_5376_HZ = 0b1001     //!< Normal (1.344 kHz) / low-power mode (5.376 KHz)
} lsm303_acc_rate_t;

/**
 * Accelerometer scales
 */
typedef enum {
    LSM303_ACC_SCALE_2G = 0b00, //!< 1 mg/LSB, +- 2G
    LSM303_ACC_SCALE_4G = 0b01, //!< 2 mg/LSB, +- 4G
    LSM303_ACC_SCALE_8G = 0b10, //!< 4 mg/LSB, +- 8G
    LSM303_ACC_SCALE_16G = 0b11 //!< 12 mg/LSB, +- 16G
} lsm303_acc_scale_t;

/**
 * Magnetometer modes
 */
typedef enum {
    LSM303_MAG_MODE_CONT = 0x00,   //!< Continuous-conversion mode
    LSM303_MAG_MODE_SINGLE = 0x01, //!< Single-conversion mode
    LSM303_MAG_MODE_SLEEP1 = 0x02, //!< Sleep-mode. Device is placed in sleep-mode
    LSM303_MAG_MODE_SLEEP2 = 0x03  //!< Sleep-mode. Device is placed in sleep-mode
} lsm303_mag_mode_t;

/**
 * Magnetometer rates
 */
typedef enum {
    LSM303_MAG_RATE_0_75, //!< 0.75 Hz
    LSM303_MAG_RATE_1_5,  //!< 1.5 Hz
    LSM303_MAG_RATE_3_0,  //!< 3.0 Hz
    LSM303_MAG_RATE_7_5,  //!< 7.5 Hz
    LSM303_MAG_RATE_15,   //!< 15 Hz
    LSM303_MAG_RATE_30,   //!< 30 Hz
    LSM303_MAG_RATE_75,   //!< 75 Hz
    LSM303_MAG_RATE_220   //!< 220 Hz
} lsm303_mag_rate_t;

/**
 * Magnetometer gains
 */
typedef enum {
    LSM303_MAG_GAIN_1_3, //!< +/- 1.3
    LSM303_MAG_GAIN_1_9, //!< +/- 1.9
    LSM303_MAG_GAIN_2_5, //!< +/- 2.5
    LSM303_MAG_GAIN_4_0, //!< +/- 4.0
    LSM303_MAG_GAIN_4_7, //!< +/- 4.7
    LSM303_MAG_GAIN_5_6, //!< +/- 5.6
    LSM303_MAG_GAIN_8_1  //!< +/- 8.1
} lsm303_mag_gain_t;

/**
 * Raw acceleration measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} lsm303_acc_raw_data_t;

/**
 * Raw magnetometer measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} lsm303_mag_raw_data_t;

/**
 * Linear acceleration measurement result: g
 */
typedef struct
{
    float x;
    float y;
    float z;
} lsm303_acc_data_t;

/**
 * Magnetic field measurement result: uT
 */
typedef struct
{
    float x;
    float y;
    float z;
} lsm303_mag_data_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev_acc;
    i2c_dev_t i2c_dev_mag;

    lsm303_acc_mode_t acc_mode;
    lsm303_acc_rate_t acc_rate;
    lsm303_acc_scale_t acc_scale;

    lsm303_mag_mode_t mag_mode;
    lsm303_mag_rate_t mag_rate;
    lsm303_mag_gain_t mag_gain;
} lsm303_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port number
 * @param acc_addr I2C accelerometer address
 * @param mag_addr I2C magnetometer address
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_init_desc(lsm303_t *dev, uint8_t acc_addr, uint8_t mag_addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_free_desc(lsm303_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_init(lsm303_t *dev);

/**
 * @brief Set accelerometer configuration
 *
 * @param dev Device descriptor
 * @param mode Power mode
 * @param rate Output data rate
 * @param scale Full scale selection
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_acc_set_config(lsm303_t *dev, lsm303_acc_mode_t mode, lsm303_acc_rate_t rate, lsm303_acc_scale_t scale);

/**
 * @brief Get accelerometer configuration
 *
 * @param dev Device descriptor
 * @param[out] mode Power mode
 * @param[out] rate Output data rate
 * @param[out] scale Full scale selection
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_acc_get_config(lsm303_t *dev, lsm303_acc_mode_t *mode, lsm303_acc_rate_t *rate, lsm303_acc_scale_t *scale);

/**
 * @brief Get accelerometer data state
 *
 * @param dev Device descriptor
 * @param[out] ready Accelerometer data ready to read if true
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_acc_data_ready(lsm303_t *dev, bool *ready);

/**
 * @brief Get raw accelerometer data
 *
 * @param dev Device descriptor
 * @param[out] raw Accelerometer data
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_acc_get_raw_data(lsm303_t *dev, lsm303_acc_raw_data_t *raw);

/**
 * @brief Convert raw accelerometer data to g
 *
 * @param dev Device descriptor
 * @param raw Raw accelerometer data
 * @param[out] data accelerometer data in g
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_acc_raw_to_g(lsm303_t *dev, lsm303_acc_raw_data_t *raw, lsm303_acc_data_t *data);

/**
 * @brief Read accelerometer data in g
 *
 * @param dev Device descriptor
 * @param[out] data accelerometer data in g
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_acc_get_data(lsm303_t *dev, lsm303_acc_data_t *data);

/**
 * @brief Set magnetometer configuration
 *
 * @param dev Device descriptor
 * @param mode Power mode
 * @param rate Output data rate
 * @param gain Gain configuration
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_mag_set_config(lsm303_t *dev, lsm303_mag_mode_t mode, lsm303_mag_rate_t rate, lsm303_mag_gain_t gain);

/**
 * @brief Get magnetometer configuration
 *
 * @param dev Device descriptor
 * @param[out] mode Power mode
 * @param[out] rate Output data rate
 * @param[out] gain Gain configuration
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_mag_get_config(lsm303_t *dev, lsm303_mag_mode_t *mode, lsm303_mag_rate_t *rate, lsm303_mag_gain_t *gain);

/**
 * @brief Get magnetometer data state
 *
 * @param dev Device descriptor
 * @param[out] ready Magnetometer data ready to read if true
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_mag_data_ready(lsm303_t *dev, bool *ready);

/**
 * @brief Get raw magnetometer data
 *
 * @param dev Device descriptor
 * @param[out] raw Magnetometer data
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_mag_get_raw_data(lsm303_t *dev, lsm303_mag_raw_data_t *raw);

/**
 * @brief Convert raw magnetometer data to uT
 *
 * @param dev Device descriptor
 * @param raw Raw magnetometer data
 * @param[out] data magnetometer data in uT
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_mag_raw_to_uT(lsm303_t *dev, lsm303_mag_raw_data_t *raw, lsm303_mag_data_t *data);

/**
 * @brief Read magnetometer data in uT
 *
 * @param dev Device descriptor
 * @param[out] data magnetometer data in uT
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_mag_get_data(lsm303_t *dev, lsm303_mag_data_t *data);

/**
 * @brief Read chip temperature
 *
 * @param dev Device descriptor
 * @param[out] temp Chip temperature
 * @return `ESP_OK` on success
 */
esp_err_t lsm303_mag_get_temp(lsm303_t *dev, float *temp);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LSM303_H__ */

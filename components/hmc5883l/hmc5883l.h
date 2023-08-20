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
 * @file hmc5883l.h
 * @defgroup hmc5883l hmc5883l
 * @{
 *
 * ESP-IDF Driver for 3-axis digital compass HMC5883L and HMC5983L
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include <stdint.h>
#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HMC5883L_ADDR 0x1e //!< I2C address

#define HMC5883L_ID 0x00333448  //!< Chip ID, "H43"

/**
 * Device operating mode
 */
typedef enum
{
    HMC5883L_MODE_CONTINUOUS = 0, //!< Continuous mode
    HMC5883L_MODE_SINGLE          //!< Single measurement mode, default
} hmc5883l_opmode_t;

/**
 * Number of samples averaged per measurement
 */
typedef enum
{
    HMC5883L_SAMPLES_1 = 0, //!< 1 sample, default
    HMC5883L_SAMPLES_2,     //!< 2 samples
    HMC5883L_SAMPLES_4,     //!< 4 samples
    HMC5883L_SAMPLES_8      //!< 8 samples
} hmc5883l_samples_averaged_t;

/**
 * Data output rate in continuous measurement mode
 */
typedef enum
{
    HMC5883L_DATA_RATE_00_75 = 0, //!< 0.75 Hz
    HMC5883L_DATA_RATE_01_50,     //!< 1.5 Hz
    HMC5883L_DATA_RATE_03_00,     //!< 3 Hz
    HMC5883L_DATA_RATE_07_50,     //!< 7.5 Hz
    HMC5883L_DATA_RATE_15_00,     //!< 15 Hz, default
    HMC5883L_DATA_RATE_30_00,     //!< 30 Hz
    HMC5883L_DATA_RATE_75_00,     //!< 75 Hz
    HMC5883L_DATA_RATE_220_00     //!< 220 Hz, HMC5983 only
} hmc5883l_data_rate_t;

/**
 * Measurement mode of the device (bias)
 */
typedef enum
{
    HMC5883L_BIAS_NORMAL = 0, //!< Default flow, no bias
    HMC5883L_BIAS_POSITIVE,   //!< Positive bias configuration all axes, used for self test (see datasheet)
    HMC5883L_BIAS_NEGATIVE    //!< Negative bias configuration all axes, used for self test (see datasheet)
} hmc5883l_bias_t;

/**
 * Device gain
 */
typedef enum
{
    HMC5883L_GAIN_1370 = 0, //!< 0.73 mG/LSb, range -0.88..+0.88 G
    HMC5883L_GAIN_1090,     //!< 0.92 mG/LSb, range -1.3..+1.3 G, default
    HMC5883L_GAIN_820,      //!< 1.22 mG/LSb, range -1.9..+1.9 G
    HMC5883L_GAIN_660,      //!< 1.52 mG/LSb, range -2.5..+2.5 G
    HMC5883L_GAIN_440,      //!< 2.27 mG/LSb, range -4.0..+4.0 G
    HMC5883L_GAIN_390,      //!< 2.56 mG/LSb, range -4.7..+4.7 G
    HMC5883L_GAIN_330,      //!< 3.03 mG/LSb, range -5.6..+5.6 G
    HMC5883L_GAIN_230       //!< 4.35 mG/LSb, range -8.1..+8.1 G
} hmc5883l_gain_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;        //!< I2C device descriptor
    hmc5883l_opmode_t opmode; //!< Operating mode
    float gain;               //!< Gain
} hmc5883l_dev_t;

/**
 * Raw measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} hmc5883l_raw_data_t;

/**
 * Measurement result, milligauss
 */
typedef struct
{
    float x;
    float y;
    float z;
} hmc5883l_data_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_init_desc(hmc5883l_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_free_desc(hmc5883l_dev_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_init(hmc5883l_dev_t *dev);

/**
 * @brief Get operating mode
 *
 * @param dev Device descriptor
 * @param[out] val Operating mode
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_get_opmode(hmc5883l_dev_t *dev, hmc5883l_opmode_t *val);

/**
 * @brief Set operating mode
 *
 * @param dev Device descriptor
 * @param mode Operating mode
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_set_opmode(hmc5883l_dev_t *dev, hmc5883l_opmode_t mode);

/**
 * @brief Get number of samples averaged per measurement output
 *
 * @param dev Device descriptor
 * @param[out] val Number of samples
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_get_samples_averaged(hmc5883l_dev_t *dev, hmc5883l_samples_averaged_t *val);

/**
 * @brief Set number of samples averaged per measurement output
 *
 * @param dev Device descriptor
 * @param samples Number of samples
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_set_samples_averaged(hmc5883l_dev_t *dev, hmc5883l_samples_averaged_t samples);

/**
 * @brief Get data output rate in continuous measurement mode
 *
 * @param dev Device descriptor
 * @param[out] val Data output rate
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_get_data_rate(hmc5883l_dev_t *dev, hmc5883l_data_rate_t *val);

/**
 * @brief Set data output rate in continuous measurement mode
 *
 * @param dev Device descriptor
 * @param rate Data output rate
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_set_data_rate(hmc5883l_dev_t *dev, hmc5883l_data_rate_t rate);

/**
 * @brief Get measurement mode (bias of the axes)
 *
 * See datasheet for self test description.
 *
 * @param dev Device descriptor
 * @param[out] val Bias
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_get_bias(hmc5883l_dev_t *dev, hmc5883l_bias_t *val);

/**
 * @brief Set measurement mode (bias of the axes)
 *
 * See datasheet for self test description.
 *
 * @param dev Device descriptor
 * @param bias Bias
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_set_bias(hmc5883l_dev_t *dev, hmc5883l_bias_t bias);

/**
 * @brief Get device gain
 *
 * @param dev Device descriptor
 * @param[out] val Current gain
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_get_gain(hmc5883l_dev_t *dev, hmc5883l_gain_t *val);

/**
 * @brief Set device gain
 * @param dev Device descriptor
 * @param gain Gain
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_set_gain(hmc5883l_dev_t *dev, hmc5883l_gain_t gain);

/**
 * @brief Get data state
 *
 * @param dev Device descriptor
 * @param[out] val true when data is written to all six data registers
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_data_is_ready(hmc5883l_dev_t *dev, bool *val);

/**
 * @brief Get lock state
 *
 * If data is locked, any new data will not be placed in data registers until
 * one of these conditions are met:
 *  - data have been read,
 *  - operating mode is changed,
 *  - the measurement configuration (bias) is changed,
 *  - power is reset.
 *
 * @param dev Device descriptor
 * @param[out] val true when data registers is locked
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_data_is_locked(hmc5883l_dev_t *dev, bool *val);

/**
 * @brief Get raw magnetic data
 *
 * @param dev Device descriptor
 * @param[out] data Raw data
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_get_raw_data(hmc5883l_dev_t *dev, hmc5883l_raw_data_t *data);

/**
 * @brief Convert raw magnetic data to milligausses
 *
 * @param dev Device descriptor
 * @param raw Source raw data
 * @param[out] mg Converted data
 */
esp_err_t hmc5883l_raw_to_mg(const hmc5883l_dev_t *dev, const hmc5883l_raw_data_t *raw, hmc5883l_data_t *mg);

/**
 * @brief Get magnetic data in milligausses
 *
 * @param dev Device descriptor
 * @param[out] data Magnetic data
 * @return `ESP_OK` on success
 */
esp_err_t hmc5883l_get_data(hmc5883l_dev_t *dev, hmc5883l_data_t *data);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __HMC5883L_H__ */

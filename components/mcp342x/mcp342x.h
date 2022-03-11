/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file mcp342x.h
 * @defgroup mcp342x mcp342x
 * @{
 *
 * ESP-IDF driver for 18-Bit, multi-channel delta-sigma
 * ADC MCP3426/MCP3427/MCP3428
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP342X_H__
#define __MCP342X_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP342X_ADDR_MIN 0x68
#define MCP342X_ADDR_MAX 0x6f

/**
 * Device operation mode
 */
typedef enum {
    MCP342X_ONESHOT = 0, //!< One-shot conversion mode
    MCP342X_CONTINUOUS   //!< Continuous conversions mode, default
} mcp342x_mode_t;

/**
 * Input channel
 */
typedef enum {
    MCP342X_CHANNEL1 = 0, //!< Channel 1, default
    MCP342X_CHANNEL2,     //!< Channel 2
    MCP342X_CHANNEL3,     //!< Channel 3 (MCP3428 only, treated as channel 1 by the MCP3426/MCP3427)
    MCP342X_CHANNEL4      //!< Channel 4 (MCP3428 only, treated as channel 2 by the MCP3426/MCP3427)
} mcp342x_channel_t;

/**
 * Resolution
 */
typedef enum {
    MCP342X_RES_12 = 0, //!< 12 bits, 240 samples per second
    MCP342X_RES_14,     //!< 14 bits, 60 samples per second
    MCP342X_RES_16,     //!< 16 bits, 15 samples per second
    MCP342X_RES_18      //!< 18 bits, 3.75 samples per second
} mcp342x_resolution_t;

/**
 * PGA gain
 */
typedef enum {
    MCP342X_GAIN1 = 0,//!< x1, default
    MCP342X_GAIN2,    //!< x2
    MCP342X_GAIN4,    //!< x4
    MCP342X_GAIN8     //!< x8
} mcp342x_gain_t;

/**
 * Device descriptor
 */
typedef struct {
    i2c_dev_t i2c_dev;               //!< I2C device descriptor
    mcp342x_mode_t mode;             //!< Operational mode
    mcp342x_channel_t channel;       //!< Input channel
    mcp342x_resolution_t resolution; //!< Resolution
    mcp342x_gain_t gain;             //!< PGA gain
} mcp342x_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param addr Device address
 * @param sda_gpio SDA GPIO pin
 * @param scl_gpio SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_init_desc(mcp342x_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_free_desc(mcp342x_t *dev);

/**
 * @brief Configure device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_set_config(mcp342x_t *dev);

/**
 * @brief Read device configuration
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_get_config(mcp342x_t *dev);

/**
 * @brief Get conversion time in microseconds
 *
 * @param dev Device descriptor
 * @param[out] us Conversion time, us
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_get_sample_time_us(mcp342x_t *dev, uint32_t *us);

/**
 * @brief Start conversion
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_start_conversion(mcp342x_t *dev);

/**
 * @brief Get raw ADC value
 *
 * @param dev Device descriptor
 * @param[out] data ADC value
 * @param[out] ready Data validity flag
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_get_data(mcp342x_t *dev, int32_t *data, bool *ready);

/**
 * @brief Get ADC voltage
 *
 * @param dev Device descriptor
 * @param[out] volts ADC voltage, volts
 * @param[out] ready Data validity flag
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_get_voltage(mcp342x_t *dev, float *volts, bool *ready);

/**
 * @brief Do a single conversion
 *
 * - start conversion
 * - wait conversion time
 * - read conversion result
 *
 * @param dev Device descriptor
 * @param[out] data ADC value
 * @return `ESP_OK` on success
 */
esp_err_t mcp342x_oneshot_conversion(mcp342x_t *dev, int32_t *data);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MCP342X_H__ */

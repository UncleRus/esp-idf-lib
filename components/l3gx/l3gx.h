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
 * @file l3gx.h
 * @defgroup l3gx l3gx
 * @{
 *
 * ESP-IDF Driver for L3GDx: L3G4200D and L3GD20 3-axis gyroscope sensors
 *
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __L3GX_H__
#define __L3GX_H__

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

#define L3G4200D_I2C_ADDR_DEF 0x68
#define L3GD20_I2C_ADDR_DEF   0x6A

/**
 * Sensor type
 */
typedef enum { L3GX_TYPE_L3G4200D, L3GX_TYPE_L3GD20, L3GX_TYPE_UNKNOWN } l3gx_sensor_type_t;

/**
 * Scales
 */
typedef enum {
    L3GX_SCALE_250 = 0b00, // full scale to 250 dps
    L3GX_SCALE_500 = 0b01, // full scale to 500 dps
    L3GX_SCALE_2000 = 0b10 // full scale to 2000 dps
} l3gx_scale_t;

/**
 * Data rates and bandwith
 */
typedef enum {
    L3GX_DRBW_100_125 = 0, // 100 Hz ODR, 12.5 Hz bandwidth
    L3GX_DRBW_100_25a,
    L3GX_DRBW_100_25b,
    L3GX_DRBW_100_25c,
    L3GX_DRBW_200_125,
    L3GX_DRBW_200_25,
    L3GX_DRBW_200_50,
    L3GX_DRBW_200_70,
    L3GX_DRBW_400_20,
    L3GX_DRBW_400_25,
    L3GX_DRBW_400_50,
    L3GX_DRBW_400_110,
    L3GX_DRBW_800_30,
    L3GX_DRBW_800_35,
    L3GX_DRBW_800_50,
    L3GX_DRBW_800_110 // 800 Hz ODR, 110 Hz bandwidth
} l3gx_drbw_t;

/**
 * Raw measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} l3gx_raw_data_t;

/**
 * Measurement result, degrees per second
 */
typedef struct
{
    float x;
    float y;
    float z;
} l3gx_data_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    l3gx_sensor_type_t sensor_type;
    l3gx_scale_t scale;
    l3gx_drbw_t datarate_bandwith;
} l3gx_t;

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
esp_err_t l3gx_init_desc(l3gx_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t l3gx_free_desc(l3gx_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t l3gx_init(l3gx_t *dev);

/**
 * @brief Read chip ID
 *
 * @param dev Device descriptor
 * @param[out] id Chip ID
 * @return `ESP_OK` on success
 */
esp_err_t l3gd20_get_chip_id(l3gx_t *dev, uint8_t *id);

/**
 * @brief Set scaling factor
 *
 * @param dev Device descriptor
 * @param[in] scale Scale factor
 * @return `ESP_OK` on success
 */
esp_err_t l3gd20_set_scale(l3gx_t *dev, l3gx_scale_t scale);

/**
 * @brief Set data rate and bandwith
 *
 * @param dev Device descriptor
 * @param[in] drbw Data rate and bandwidth setup
 * @return `ESP_OK` on success
 */
esp_err_t l3gd20_set_datarate_and_bandwith(l3gx_t *dev, l3gx_drbw_t drbw);

/**
 * @brief Get gyro data state
 *
 * @param dev Device descriptor
 * @param[out] ready Gyro data ready to read if true
 * @return `ESP_OK` on success
 */
esp_err_t l3gx_data_ready(l3gx_t *dev, bool *ready);

/**
 * @brief Read raw gyro data
 *
 * @param dev Device descriptor
 * @param[out] raw Raw gyro data
 * @return `ESP_OK` on success
 */
esp_err_t l3gx_get_raw_data(l3gx_t *dev, l3gx_raw_data_t *raw);

/**
 * @brief Convert raw gyro data to dps [degrees per second]
 *
 * @param dev Device descriptor
 * @param[in] raw Raw gyro data
 * @param[out] data Gyro data in dps
 * @return `ESP_OK` on success
 */
esp_err_t l3gd20_raw_to_dps(l3gx_t *dev, l3gx_raw_data_t *raw, l3gx_data_t *data);

/**
 * @brief Read gyro data in degrees per second
 *
 * @param dev Device descriptor
 * @param[out] data Gyro data in dps
 * @return `ESP_OK` on success
 */
esp_err_t l3gx_get_data(l3gx_t *dev, l3gx_data_t *data);

/**
 * @brief Read raw temperature data (see datasheet)
 *
 * @param dev Device descriptor
 * @param[out] temp Raw temperature data
 * @return `ESP_OK` on success
 */
esp_err_t l3gx_get_raw_temp(l3gx_t *dev, int16_t *temp);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __L3GX_H__ */

/*
 * Copyright (c) 2022 Jan Veeh (jan.veeh@motius.de)
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
 * @file icm42670.h
 * @defgroup icm42670 icm42670
 * @{
 *
 * ESP-IDF driver for TDK ICM-42670-P IMU (found on ESP-RS board)
 *
 * Copyright (c) 2022 Jan Veeh (jan.veeh@motius.de)
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __ICM42670_H__
#define __ICM42670_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICM42670_I2C_ADDR_GND   0x68
#define ICM42670_I2C_ADDR_VCC   0x69

// Registers USER BANK 0
#define ICM42670_REG_MCLK_RDY           0x00
#define ICM42670_REG_DEVICE_CONFIG      0x01
#define ICM42670_REG_SIGNAL_PATH_RESET  0x02
#define ICM42670_REG_DRIVE_CONFIG1      0x03
#define ICM42670_REG_DRIVE_CONFIG2      0x04
#define ICM42670_REG_DRIVE_CONFIG3      0x05
#define ICM42670_REG_INT_CONFIG         0x06
#define ICM42670_REG_TEMP_DATA1         0x09
#define ICM42670_REG_TEMP_DATA0         0x0A
#define ICM42670_REG_ACCEL_DATA_X1      0x0B
#define ICM42670_REG_ACCEL_DATA_X0      0x0C
#define ICM42670_REG_ACCEL_DATA_Y1      0x0D
#define ICM42670_REG_ACCEL_DATA_Y0      0x0E
#define ICM42670_REG_ACCEL_DATA_Z1      0x0F
#define ICM42670_REG_ACCEL_DATA_Z0      0x10
#define ICM42670_REG_GYRO_DATA_X1       0x11
#define ICM42670_REG_GYRO_DATA_X0       0x12
#define ICM42670_REG_GYRO_DATA_Y1       0x13
#define ICM42670_REG_GYRO_DATA_Y0       0x14
#define ICM42670_REG_GYRO_DATA_Z1       0x15
#define ICM42670_REG_GYRO_DATA_Z0       0x16
#define ICM42670_REG_TMST_FSYNCH        0x17
#define ICM42670_REG_TMST_FSYNCL        0x18
#define ICM42670_REG_APEX_DATA4         0x1D
#define ICM42670_REG_APEX_DATA5         0x1E
#define ICM42670_REG_PWR_MGMT0          0x1F
#define ICM42670_REG_GYRO_CONFIG0       0x20
#define ICM42670_REG_ACCEL_CONFIG0      0x21
#define ICM42670_REG_TEMP_CONFIG0       0x22
#define ICM42670_REG_GYRO_CONFIG1       0x23
#define ICM42670_REG_ACCEL_CONFIG1      0x24
#define ICM42670_REG_APEX_CONFIG0       0x25
#define ICM42670_REG_APEX_CONFIG1       0x26
#define ICM42670_REG_WOM_CONFIG         0x27
#define ICM42670_REG_FIFO_CONFIG1       0x28
#define ICM42670_REG_FIFO_CONFIG2       0x29
#define ICM42670_REG_FIFO_CONFIG3       0x2A
#define ICM42670_REG_INT_SOURCE0        0x2B
#define ICM42670_REG_INT_SOURCE1        0x2C
#define ICM42670_REG_INT_SOURCE3        0x2D
#define ICM42670_REG_INT_SOURCE4        0x2E
#define ICM42670_REG_FIFO_LOST_PKT0     0x2F
#define ICM42670_REG_FIFO_LOST_PKT1     0x30
#define ICM42670_REG_APEX_DATA0         0x31
#define ICM42670_REG_APEX_DATA1         0x32
#define ICM42670_REG_APEX_DATA2         0x33
#define ICM42670_REG_APEX_DATA3         0x34
#define ICM42670_REG_INTF_CONFIG0       0x35
#define ICM42670_REG_INTF_CONFIG1       0x36
#define ICM42670_REG_INT_STATUS_DRDY    0x39
#define ICM42670_REG_INT_STATUS         0x3A
#define ICM42670_REG_INT_STATUS2        0x3B
#define ICM42670_REG_INT_STATUS3        0x3C
#define ICM42670_REG_FIFO_COUNTH        0x3D
#define ICM42670_REG_FIFO_COUNTL        0x3E
#define ICM42670_REG_FIFO_DATA          0x3F
#define ICM42670_REG_WHO_AM_I           0x75
#define ICM42670_REG_BLK_SEL_W          0x79
#define ICM42670_REG_MADDR_W            0x7A
#define ICM42670_REG_M_W                0x7B
#define ICM42670_REG_BLK_SEL_R          0x7C
#define ICM42670_REG_MADDR_R            0x7D
#define ICM42670_REG_M_R                0x7E

/**
 * Integration time
 */
typedef enum
{
    TSL2561_INTEGRATION_13MS = 0, //!< 13ms
    TSL2561_INTEGRATION_101MS,    //!< 101ms
    TSL2561_INTEGRATION_402MS     //!< 402ms, default
} tsl2561_integration_time_t;

/**
 * Gain
 */
typedef enum
{
    TSL2561_GAIN_1X = 0x00, //!< Default
    TSL2561_GAIN_16X = 0x10
} tsl2561_gain_t;

typedef enum
{
    TSL2561_PACKAGE_CS = 0,
    TSL2561_PACKAGE_T_FN_CL
} tsl2561_package_t;

typedef enum
{
    ICM42670_GYRO_DISABLE = 0b00,
    ICM42670_GYRO_STANDBY = 0b01,
    ICM42670_GYRO_ENABLE_LN_MODE = 0b11
} icm42670_gyro_pwr_mode_t;

typedef enum
{
    ICM42670_ACCEL_DISABLE = 0b00,
    ICM42670_ACCEL_ENABLE_LP_MODE = 0b10,
    ICM42670_ACCEL_ENABLE_LN_MODE = 0b11
} icm42670_accel_pwr_mode_t;

typedef enum
{
    ICM42670_GYRO_RANGE_2000DPS = 0b00,
    ICM42670_GYRO_RANGE_1000DPS = 0b01,
    ICM42670_GYRO_RANGE_500DPS = 0b10,
    ICM42670_GYRO_RANGE_250DPS = 0b11
} icm42670_gyro_range_t;

typedef enum
{
    ICM42670_GYRO_ODR_12_5HZ = 0b1100,
    ICM42670_GYRO_ODR_25HZ = 0b1011,
    ICM42670_GYRO_ODR_50HZ = 0b1010,
    ICM42670_GYRO_ODR_100HZ = 0b1001,
    ICM42670_GYRO_ODR_200HZ = 0b1000,
    ICM42670_GYRO_ODR_400HZ = 0b0111,
    ICM42670_GYRO_ODR_800HZ = 0b0110,
    ICM42670_GYRO_ODR_1_6KHZ = 0b0101
} icm42670_gyro_odr_t;

typedef enum
{
    ICM42670_ACCEL_RANGE_16G = 0b00,
    ICM42670_ACCEL_RANGE_8G = 0b01,
    ICM42670_ACCEL_RANGE_4G = 0b10,
    ICM42670_ACCEL_RANGE_2G = 0b11
} icm42670_accel_range_t;

typedef enum
{
    ICM42670_ACCEL_ODR_1_5625HZ = 0b1111,
    ICM42670_ACCEL_ODR_3_125HZ = 0b1110,
    ICM42670_ACCEL_ODR_6_25HZ = 0b1101,
    ICM42670_ACCEL_ODR_12_5HZ = 0b1100,
    ICM42670_ACCEL_ODR_25HZ = 0b1011,
    ICM42670_ACCEL_ODR_50HZ = 0b1010,
    ICM42670_ACCEL_ODR_100HZ = 0b1001,
    ICM42670_ACCEL_ODR_200HZ = 0b1000,
    ICM42670_ACCEL_ODR_400HZ = 0b0111,
    ICM42670_ACCEL_ODR_800HZ = 0b0110,
    ICM42670_ACCEL_ODR_1_6KHZ = 0b0101
} icm42670_accel_odr_t;

typedef enum
{
    ICM42670_TEMP_LFP_BYPASSED = 0b000,
    ICM42670_TEMP_LFP_180HZ = 0b001,
    ICM42670_TEMP_LFP_72HZ = 0b010,
    ICM42670_TEMP_LFP_34HZ = 0b011,
    ICM42670_TEMP_LFP_16HZ = 0b100,
    ICM42670_TEMP_LFP_8HZ = 0b101,
    ICM42670_TEMP_LFP_4HZ = 0b110
} icm42670_temp_lfp_t;

typedef enum
{
    ICM42670_GYRO_LFP_BYPASSED = 0b000,
    ICM42670_GYRO_LFP_180HZ = 0b001,
    ICM42670_GYRO_LFP_121HZ = 0b010,
    ICM42670_GYRO_LFP_73HZ = 0b011,
    ICM42670_GYRO_LFP_53HZ = 0b100,
    ICM42670_GYRO_LFP_34HZ = 0b101,
    ICM42670_GYRO_LFP_25HZ = 0b110,
    ICM42670_GYRO_LFP_16HZ = 0b111
} icm42670_gyro_lfp_t;

typedef enum
{
    ICM42670_ACCEL_LFP_BYPASSED = 0b000,
    ICM42670_ACCEL_LFP_180HZ = 0b001,
    ICM42670_ACCEL_LFP_121HZ = 0b010,
    ICM42670_ACCEL_LFP_73HZ = 0b011,
    ICM42670_ACCEL_LFP_53HZ = 0b100,
    ICM42670_ACCEL_LFP_34HZ = 0b101,
    ICM42670_ACCEL_LFP_25HZ = 0b110,
    ICM42670_ACCEL_LFP_16HZ = 0b111
} icm42670_accel_lfp_t;

typedef enum
{
    ICM42670_ACCEL_AVG_2X = 0b000,
    ICM42670_ACCEL_AVG_4X = 0b001,
    ICM42670_ACCEL_AVG_8X = 0b010,
    ICM42670_ACCEL_AVG_16X = 0b011,
    ICM42670_ACCEL_AVG_32X = 0b100,
    ICM42670_ACCEL_AVG_64X = 0b101
} icm42670_accel_avg_t;

typedef enum
{
    ICM42670_INT_MODE_PULSED = 0,
    ICM42670_INT_MODE_LATCHED = 1
} icm42670_int_mode_t;

typedef enum
{
    ICM42670_INT_DRIVE_OPEN_DRAIN = 0,
    ICM42670_INT_DRIVE_PUSH_PULL = 1
} icm42670_int_drive_t;

typedef enum
{
    ICM42670_INT_POLARITY_ACTIVE_LOW = 0,
    ICM42670_INT_POLARITY_ACTIVE_HIGH = 1
} icm42670_int_polarity_t;

typedef struct
{
    icm42670_int_mode_t mode;
    icm42670_int_drive_t drive;
    icm42670_int_polarity_t polarity;
} icm42670_int_config_t;


/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    // TODO
} icm42670_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr I2C device address, `TSL2561_I2C_ADDR_...` const
 * @param port I2C port
 * @param sda_gpio SDA GPIO pin
 * @param scl_gpio SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_init_desc(icm42670_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_free_desc(icm42670_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_init(icm42670_t *dev);

/**
 * @brief Set device integration time
 *
 * @param dev Device descriptor
 * @param integration_time Integration time
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_integration_time(icm42670_t *dev, tsl2561_integration_time_t integration_time);

/**
 * @brief Set gyro power mode
 *
 * @param dev Device descriptor
 * @param pwr_mode icm42670_gyro_pwr_mode_t power mode
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_gyro_pwr_mode(icm42670_t *dev, icm42670_gyro_pwr_mode_t pwr_mode);

/**
 * @brief Set accel power mode
 *
 * @param dev Device descriptor
 * @param pwr_mode icm42670_accel_pwr_mode_t power mode
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_accel_pwr_mode(icm42670_t *dev, icm42670_accel_pwr_mode_t pwr_mode);

/**
 * @brief Read temperature from device
 *
 * @param dev Device descriptor
 * @param[out] temperature temperature, degree C
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_read_temperature(icm42670_t *dev, float *temperature);

/**
 * @brief Read raw data registers
 *
 * @param dev Device descriptor
 * @param data_register data register to read from
 * @param[out] data accel or gyro data
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_read_raw_data(icm42670_t *dev, uint8_t data_register, uint16_t *data);

esp_err_t icm42670_set_gyro_pwr_mode(icm42670_t *dev, icm42670_gyro_pwr_mode_t pwr_mode);

esp_err_t icm42670_set_accel_pwr_mode(icm42670_t *dev, icm42670_accel_pwr_mode_t pwr_mode);

esp_err_t icm42670_read_raw_data(icm42670_t *dev, uint8_t data_register, uint16_t *data);

esp_err_t icm42670_read_temperature(icm42670_t *dev, float *temperature);

esp_err_t icm42670_reset(icm42670_t *dev);

esp_err_t icm42670_flush_fifo(icm42670_t *dev);

esp_err_t icm42670_set_gyro_range(icm42670_t *dev, icm42670_gyro_range_t range);

esp_err_t icm42670_set_gyro_odr(icm42670_t *dev, icm42670_gyro_odr_t odr);

esp_err_t icm42670_set_accel_range(icm42670_t *dev, icm42670_accel_range_t range);

esp_err_t icm42670_set_accel_odr(icm42670_t *dev, icm42670_accel_odr_t odr);

esp_err_t icm42670_set_int_config(icm42670_t *dev, uint8_t int_pin, icm42670_int_config_t config);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __ICM42670_H__

/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2025 Lukasz Bielinski <lbielinski01@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __BMI160_H__
#define __BMI160_H__


#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#include "bmi160_reg.h"


#ifdef __cplusplus
extern "C" {
#endif

#define BMI160_I2C_ADDRESS_GND 0x68u ///< I2C address of BMI160 if SDO is pulled to GND
#define BMI160_I2C_ADDRESS_VDD 0x69u ///< I2C address of BMI160 if SDO is pulled to VDDIO

/**
 * @brief enum for the accelerometer range
 * 
 */
typedef enum 
{
    BMI160_ACC_RANGE_2G = 0x03u, ///< +-2g
    BMI160_ACC_RANGE_4G = 0x05u, ///< +-4g
    BMI160_ACC_RANGE_8G = 0x08u, ///< +-8g
    BMI160_ACC_RANGE_16G = 0x0Cu ///< +-16g
}bmi160_acc_range_t;

/**
 * @brief enum for the gyroscope range
 * 
 */
typedef enum 
{
    BMI160_GYR_RANGE_125DPS = 0x02u, ///< +-125dps
    BMI160_GYR_RANGE_250DPS = 0x00u, ///< +-250dps
    BMI160_GYR_RANGE_500DPS = 0x04u, ///< +-500dps
    BMI160_GYR_RANGE_1000DPS = 0x06u, ///< +-1000dps
    BMI160_GYR_RANGE_2000DPS = 0x08u ///< +-2000dps
}bmi160_gyr_range_t;

/**
 * @brief enum for the accelerometer ODR
 * 
 */
typedef enum
{
    BMI160_ACC_ODR_0_78HZ = 0x01u, ///< 0.78Hz
    BMI160_ACC_ODR_1_56HZ = 0x02u, ///< 1.56Hz
    BMI160_ACC_ODR_3_12HZ = 0x03u, ///< 3.12Hz
    BMI160_ACC_ODR_6_25HZ = 0x04u, ///< 6.25Hz
    BMI160_ACC_ODR_12_5HZ = 0x05u, ///< 12.5Hz
    BMI160_ACC_ODR_25HZ = 0x06u, ///< 25Hz
    BMI160_ACC_ODR_50HZ = 0x07u, ///< 50Hz
    BMI160_ACC_ODR_100HZ = 0x08u, ///< 100Hz
    BMI160_ACC_ODR_200HZ = 0x09u, ///< 200Hz
    BMI160_ACC_ODR_400HZ = 0x0Au, ///< 400Hz
    BMI160_ACC_ODR_800HZ = 0x0Bu, ///< 800Hz
    BMI160_ACC_ODR_1600HZ = 0x0Cu ///< 1600Hz
}bmi160_acc_odr_t;

/**
 * @brief enum for the gyroscope ODR
 * 
 */
typedef enum
{
    BMI160_GYR_ODR_25HZ = 0x06u, ///< 25Hz
    BMI160_GYR_ODR_50HZ = 0x07u, ///< 50Hz
    BMI160_GYR_ODR_100HZ = 0x08u, ///< 100Hz
    BMI160_GYR_ODR_200HZ = 0x09u, ///< 200Hz
    BMI160_GYR_ODR_400HZ = 0x0Au, ///< 400Hz
    BMI160_GYR_ODR_800HZ = 0x0Bu, ///< 800Hz
    BMI160_GYR_ODR_1600HZ = 0x0Cu ///< 1600Hz
}bmi160_gyr_odr_t;

/**
 * @brief enum for int pin
 * 
 */
typedef enum
{
    BMI160_PIN_INT1 = 0x00u, ///< INT1
    BMI160_PIN_INT2 = 0x01u ///< INT2
}bmi160_int_pin_t;

/**
 * @brief enum for int enable
 * 
 */
typedef enum
{
    BMI160_INT_DISABLE = 0x00u, ///< Disable interrupt
    BMI160_INT_ENABLE = 0x01u ///< Enable interrupt
}bmi160_int_enable_t;

/**
 * @brief enum for int od
 * 
 */
typedef enum
{
    BMI160_INT_PUSH_PULL = 0x00u, ///< Push-pull
    BMI160_INT_OPEN_DRAIN = 0x01u ///< Open-drain
}bmi160_int_od_t;

/**
 * @brief enum for int level
 * 
 */
typedef enum
{
    BMI160_INT_ACTIVE_LOW = 0x00u, ///< Active low
    BMI160_INT_ACTIVE_HIGH = 0x01u ///< Active high
}bmi160_int_level_t; 

/**
 * @brief step counter mode
 * 
 */
typedef enum
{
    BMI160_STEP_COUNTER_NORMAL = 0x00u, ///< Normal mode
    BMI160_STEP_COUNTER_SENSITIVE = 0x01u, ///< Sensitive mode
    BMI160_STEP_COUNTER_ROBUST = 0x02u ///< Robust mode
}bmi160_step_counter_mode_t;

/**
 * @brief Accelerometer PMU mode
 * 
 */
typedef enum
{
    BMI160_PMU_ACC_SUSPEND = 0x10u, ///< Suspend mode
    BMI160_PMU_ACC_NORMAL = 0x11u, ///< Normal mode
    BMI160_PMU_ACC_LOW_POWER = 0x12u ///< Low power mode
}bmi160_pmu_acc_mode_t;

/**
 * @brief Gyroscope PMU mode
 * 
 */
typedef enum
{
    BMI160_PMU_GYR_SUSPEND = 0x14u, ///< Suspend mode
    BMI160_PMU_GYR_NORMAL = 0x15u, ///< Normal mode
    BMI160_PMU_GYR_FAST_STARTUP = 0x17u, ///< Fast startup mode
}bmi160_pmu_gyr_mode_t;

/**
 * @brief averaging configuration in low power mode for accelerometer
 * 
 * @note For low power mode defines averaging, for normal mode defines filter bandwidth
 * 
 */
typedef enum
{
    BMI160_ACC_LP_AVG_1 = 0x00u, ///< 1 sample
    BMI160_ACC_LP_AVG_2 = 0x01u, ///< 2 samples
    BMI160_ACC_LP_AVG_4 = 0x02u, ///< 4 samples
    BMI160_ACC_LP_AVG_8 = 0x03u, ///< 8 samples
    BMI160_ACC_LP_AVG_16 = 0x04u, ///< 16 samples
    BMI160_ACC_LP_AVG_32 = 0x05u, ///< 32 samples
    BMI160_ACC_LP_AVG_64 = 0x06u, ///< 64 samples
    BMI160_ACC_LP_AVG_128 = 0x07u ///< 128 samples
}bmi160_acc_lp_avg_t;

/**
 * @brief bmi160 accelerometer undersampling configuration
 * 
 * @note This is only applicable for low power mode
 */
typedef enum
{
    BMI160_ACC_US_ON = 0x01u, ///< Enable undersampling
    BMI160_ACC_US_OFF = 0x00u ///< Disable undersampling
}bmi160_acc_us_t;

/**
 * @brief tap configuration quiet
 * 
 */
typedef enum
{
    BMI160_TAP_QUIET_30MS = 0x00u, ///< 30ms
    BMI160_TAP_QUIET_20MS = 0x01u ///< 20ms
}bmi160_tap_quiet_t;

/**
 * @brief tap mode
 * 
 */
typedef enum
{
    BMI160_TAP_MODE_SINGLE = 0x00u, ///< Single tap
    BMI160_TAP_MODE_DOUBLE = 0x01u, ///< Double tap
}bmi160_tap_mode_t;

/**
 * @brief tap configuration shock
 * 
 */
typedef enum
{
    BMI160_TAP_SHOCK_50MS = 0x00u, ///< 50ms
    BMI160_TAP_SHOCK_75MS = 0x01u ///< 75ms
}bmi160_tap_shock_t;

/**
 * @brief tap configuration duration
 * 
 */
typedef enum
{
    BMI160_TAP_DUR_50MS = 0x00u, ///< 50ms
    BMI160_TAP_DUR_100MS = 0x01u, ///< 100ms
    BMI160_TAP_DUR_150MS = 0x02u,
    BMI160_TAP_DUR_200MS = 0x03u,
    BMI160_TAP_DUR_250MS = 0x04,
    BMI160_TAP_DUR_375MS = 0x05,
    BMI160_TAP_DUR_500MS = 0x06,
    BMI160_TAP_DUR_700MS = 0x07
}bmi160_tap_dur_t;

/**
 * @brief tap threshold
 * 
 */
typedef enum
{
    BMI160_TAP_TH_0_032G = 0x00u, ///< 0.032g
    BMI160_TAP_TH_0_0625G = 0x01u, ///< 0.0625g
    BMI160_TAP_TH_0_125G = 0x02u, ///< 0.125g
    BMI160_TAP_TH_0_25G = 0x03u, ///< 0.25g
    BMI160_TAP_TH_0_5G = 0x04u, ///< 0.5g
    BMI160_TAP_TH_1G = 0x05u, ///< 1g
    BMI160_TAP_TH_2G = 0x06u, ///< 2g
    BMI160_TAP_TH_4G = 0x07u ///< 4g
}bmi160_tap_th_t;

/**
 * @brief bmi160 configuration structure
 * 
 */
typedef struct
{
    bmi160_acc_range_t accRange; ///< Accelerometer range
    bmi160_acc_odr_t accOdr; ///< Accelerometer ODR
    bmi160_pmu_acc_mode_t accMode; ///< Accelerometer PMU mode
    bmi160_acc_lp_avg_t accAvg; ///< Accelerometer averaging configuration for low power mode
    bmi160_acc_us_t accUs; ///< Accelerometer undersampling configuration
    bmi160_gyr_range_t gyrRange; ///< Gyroscope range
    bmi160_gyr_odr_t gyrOdr; ///< Gyroscope ODR
    bmi160_pmu_gyr_mode_t gyrMode; ///< Gyroscope PMU mode
}bmi160_conf_t;

/**
 * @brief result structure for the BMI160
 * 
 */
typedef struct
{
    float accX; ///< Accelerometer X axis
    float accY; ///< Accelerometer Y axis
    float accZ; ///< Accelerometer Z axis
    float gyroX; ///< Gyroscope X axis
    float gyroY; ///< Gyroscope Y axis
    float gyroZ; ///< Gyroscope Z axis
}bmi160_result_t;

/**
 * @brief int out configuration
 * 
 */
typedef struct
{
    bmi160_int_pin_t intPin; ///< Interrupt pin
    bmi160_int_enable_t intEnable; ///< Enable interrupt
    bmi160_int_od_t intOd; ///< Open-drain
    bmi160_int_level_t intLevel; ///< Active high
}bmi160_int_out_conf_t;

/**
 * @brief tap configuration
 * 
 */
typedef struct
{
    bmi160_tap_quiet_t tapQuiet; ///< Tap quiet
    bmi160_tap_shock_t tapShock; ///< Tap shock
    bmi160_tap_dur_t tapDur; ///< Tap duration
    bmi160_tap_th_t tapTh; ///< Tap threshold
    bmi160_tap_mode_t tapMode; ///< Tap mode
}bmi160_tap_conf_t;

/**
 *  Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;              ///< I2C device descriptor
    float aBias[3];                 ///< Accelerometer bias
    float gBias[3];                 ///< Gyroscope bias
    float aRes;                     ///< Accelerometer resolution
    float gRes;                     ///< Gyroscope resolution
    bmi160_acc_range_t accRange;    ///< Accelerometer range
    bmi160_gyr_range_t gyrRange;    ///< Gyroscope range
    bmi160_acc_odr_t accOdr;        ///< Accelerometer ODR
    uint8_t accConf;                ///< Accelerometer configuration
    bmi160_gyr_odr_t gyrOdr;        ///< Gyroscope ODR
    gpio_num_t intPin;              ///< Interrupt pin
    bmi160_tap_mode_t   tapMode;   ///< Tap mode
    void (*callback)();        ///< Interrupt callback
} bmi160_t;

/**
 * @brief Initialize the BMI160
 * 
 * @param dev Pointer to the device descriptor
 * @param addr I2C address of the device
 * @param port I2C port number
 * @param sda_gpio SDA GPIO number
 * @param scl_gpio SCL GPIO number
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_init(bmi160_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Read a register from the BMI160
 * 
 * @param dev Pointer to the device descriptor
 * @param reg Register address
 * @param val Pointer to store the value
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_read_reg(bmi160_t *dev, uint8_t reg, uint8_t *val);

/**
 * @brief Read an array of registers from the BMI160
 * 
 * @param dev Pointer to the device descriptor
 * @param reg Register address
 * @param val Pointer to store the values
 * @param num Number of registers to read
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_read_reg_array(bmi160_t *dev, uint8_t reg, uint8_t *val, uint8_t num);

/**
 * @brief free descriptor
 * 
 * @param dev Pointer to the device descriptor
 * @return esp_err_t ESP_OK if success
 * 
 */
esp_err_t bmi160_free(bmi160_t *dev);

/**
 * @brief Write a register to the BMI160
 * 
 * @param dev Pointer to the device descriptor
 * @param reg Register address
 * @param val Value to write
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_write_reg(bmi160_t *dev, uint8_t reg, uint8_t val);

/**
 * @brief Calibrate the BMI160 to calculate bias
 * 
 * @param dev Pointer to the device descriptor
 * @return esp_err_t ESP_OK if success
 * 
 * @note bmi160_start must be called before calling this function. After calling this function output data will be offset by the bias values.
 */
esp_err_t bmi160_calibrate(bmi160_t *dev);

/**
 * @brief start the BMI160
 * 
 * @param dev Pointer to the device descriptor
 * @param conf Pointer to the configuration structure
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_start(bmi160_t *dev, bmi160_conf_t* conf);

/**
 * @brief Read data from the BMI160
 * 
 * @param dev Pointer to the device descriptor
 * @param result Pointer to store the result
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_read_data(bmi160_t *dev, bmi160_result_t *result);

/**
 * @brief set acc range
 * 
 * @param dev Pointer to the device descriptor
 * @param range Accelerometer range
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_set_acc_range(bmi160_t *dev, bmi160_acc_range_t range);

/**
 * @brief set gyro range
 * 
 * @param dev Pointer to the device descriptor
 * @param range Gyroscope range
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_set_gyr_range(bmi160_t *dev, bmi160_gyr_range_t range);

/**
 * @brief set acc configuration
 * 
 * @param dev Pointer to the device descriptor
 * @param odr Accelerometer ODR
 * @param avg Averaging configuration
 * @param acc_us Accelerometer undersampling configuration
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_set_acc_conf(bmi160_t *dev, bmi160_acc_odr_t odr, bmi160_acc_lp_avg_t avg, bmi160_acc_us_t acc_us);

/**
 * @brief set gyro odr
 * 
 * @param dev Pointer to the device descriptor
 * @param odr Gyroscope ODR
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_set_gyr_odr(bmi160_t *dev, bmi160_gyr_odr_t odr);

/**
 * @brief self test for the BMI160
 * 
 * @param dev Pointer to the device descriptor
 * @return esp_err_t ESP_OK if success
 *  
 */
esp_err_t bmi160_self_test(bmi160_t *dev);

/**
 * @brief Enable interrupt for the BMI160
 * 
 * @param dev Pointer to the device descriptor
 * @param intOutConf Interrupt out configuration
 * @return esp_err_t ESP_OK if success
 * 
 */
esp_err_t bmi160_enable_int_new_data(bmi160_t *dev, bmi160_int_out_conf_t* intOutConf);

/**
 * @brief configure step counter
 * 
 * @param dev Pointer to the device descriptor
 * @param mode Step counter mode
 * 
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_enable_step_counter(bmi160_t *dev, bmi160_step_counter_mode_t mode);

/**
 * @brief read step counter
 * 
 * @param dev Pointer to the device descriptor
 * @param stepCounter Pointer to store the step counter value
 * 
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_read_step_counter(bmi160_t *dev, uint16_t *stepCounter);

/**
 * @brief reset step counter
 * 
 * @param dev Pointer to the device descriptor
 * 
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_reset_step_counter(bmi160_t *dev);

/**
 * @brief Enable interrupt for the BMI160 for new step
 * 
 * @param dev Pointer to the device descriptor
 * @param intOutConf Interrupt out configuration
 * @return esp_err_t ESP_OK if success
 * 
 */
esp_err_t bmi160_enable_int_step(bmi160_t *dev, bmi160_int_out_conf_t* intOutConf);

/**
 * @brief switch mode for the BMI160
 * 
 */
esp_err_t bmi160_switch_mode(bmi160_t *dev, bmi160_pmu_acc_mode_t accMode, bmi160_pmu_gyr_mode_t gyrMode);

/**
 * @brief configure tap detection
 * 
 * @param dev Pointer to the device descriptor
 * @param tapConf Pointer to the configuration structure
 * 
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_enable_tap_detection(bmi160_t *dev, bmi160_tap_conf_t* tapConf);

/**
 * @brief Enable interrupt for the BMI160 for tap detection
 * 
 * @param dev Pointer to the device descriptor
 * @param intOutConf Interrupt out configuration
 * 
 * @return esp_err_t ESP_OK if success
 * 
 */
esp_err_t bmi160_enable_int_tap(bmi160_t *dev, bmi160_int_out_conf_t* intOutConf);

/**
 * @brief read tap orientation
 * 
 * @param dev Pointer to the device descriptor
 * @param orient Pointer to store the orientation
 * 
 * @return esp_err_t ESP_OK if success
 */
esp_err_t bmi160_read_tap_orient(bmi160_t *dev, uint8_t *orient);

#ifdef __cplusplus
}
#endif

#endif /* __BMI160_H__ */

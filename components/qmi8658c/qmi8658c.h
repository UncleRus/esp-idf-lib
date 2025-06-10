/*
 * Copyright (c) 2024 xyzroe <i@xyzroe.cc>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
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
 * @file qmi8658c.h
 * @brief Header file for QMI8658C sensor driver
 * @author xyzroe
 * ESP-IDF driver for QMI8658C sensor
 *
 * Datasheet: https://qstcorp.com/upload/pdf/202202/QMI8658C%20datasheet%20rev%200.9.pdf
 *
 * Copyright (c) 2024 xyzroe <i@xyzroe.cc>
 */

#ifndef __QMI8658C_H__
#define __QMI8658C_H__

#include <stdint.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAUL_QMI8658C_ADDR 0x6B //!< I2C address for QMI8658C
#define QMI8658C_I2C_FREQ_HZ 400000

/* General purpose registers */
#define QMI8658_WHO_AM_I 0x00 // WHO_AM_I register address.
#define QMI8658_REVISION 0x01 // REVISION register address.

/* Setup and control registers */
#define QMI8658_CTRL1 0x02 // Control register 1 address.
#define QMI8658_CTRL2 0x03 // Control register 2 address.
#define QMI8658_CTRL3 0x04 // Control register 3 address.
#define QMI8658_CTRL4 0x05 // Control register 4 address.
#define QMI8658_CTRL5 0x06 // Control register 5 address.
#define QMI8658_CTRL6 0x07 // Control register 6 address.
#define QMI8658_CTRL7 0x08 // Control register 7 address.
#define QMI8658_CTRL9 0x0A // Control register 9 address.

/* Data output registers */

// Accelerometer
#define QMI8658_ACC_X_L 0x35 // Accelerometer X-axis low byte.
#define QMI8658_ACC_X_H 0x36 // Accelerometer X-axis high byte.
#define QMI8658_ACC_Y_L 0x37 // Accelerometer Y-axis low byte.
#define QMI8658_ACC_Y_H 0x38 // Accelerometer Y-axis high byte.
#define QMI8658_ACC_Z_L 0x39 // Accelerometer Z-axis low byte.
#define QMI8658_ACC_Z_H 0x3A // Accelerometer Z-axis high byte.

// Gyroscope
#define QMI8658_GYR_X_L 0x3B // Gyroscope X-axis low byte.
#define QMI8658_GYR_X_H 0x3C // Gyroscope X-axis high byte.
#define QMI8658_GYR_Y_L 0x3D // Gyroscope Y-axis low byte.
#define QMI8658_GYR_Y_H 0x3E // Gyroscope Y-axis high byte.
#define QMI8658_GYR_Z_L 0x3F // Gyroscope Z-axis low byte.
#define QMI8658_GYR_Z_H 0x40 // Gyroscope Z-axis high byte.

// Temperature sensor
#define QMI8658_TEMP_L 0x33 // Temperature sensor low byte.
#define QMI8658_TEMP_H 0x34 // Temperature sensor high byte.

/* Soft reset register */
#define QMI8658_RESET 0x60 // Soft reset register address.

/* define scale sensitivity */
/* Accelerometer scale sensitivity values for different gravity ranges */
#define ACC_SCALE_SENSITIVITY_2G  (1 << 14) // Sensitivity for ±2g range.
#define ACC_SCALE_SENSITIVITY_4G  (1 << 13) // Sensitivity for ±4g range.
#define ACC_SCALE_SENSITIVITY_8G  (1 << 12) // Sensitivity for ±8g range.
#define ACC_SCALE_SENSITIVITY_16G (1 << 11) // Sensitivity for ±16g range.

/* Gyroscope scale sensitivity values for different degrees per second ranges */
#define GYRO_SCALE_SENSITIVITY_16DPS   (1 << 11) // Sensitivity for ±16 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_32DPS   (1 << 10) // Sensitivity for ±32 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_64DPS   (1 << 9)  // Sensitivity for ±64 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_128DPS  (1 << 8)  // Sensitivity for ±128 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_256DPS  (1 << 7)  // Sensitivity for ±256 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_512DPS  (1 << 6)  // Sensitivity for ±512 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_1024DPS (1 << 5)  // Sensitivity for ±1024 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_2048DPS (1 << 4)  // Sensitivity for ±2048 degrees per second range.

typedef enum {
    QMI8658C_MODE_ACC_ONLY = 1,
    QMI8658C_MODE_GYRO_ONLY,
    QMI8658C_MODE_DUAL,
} qmi8658c_mode_t;

typedef enum {
    QMI8658C_ACC_ODR_8000,
    QMI8658C_ACC_ODR_4000,
    QMI8658C_ACC_ODR_2000,
    QMI8658C_ACC_ODR_1000,
    QMI8658C_ACC_ODR_500,
    QMI8658C_ACC_ODR_250,
    QMI8658C_ACC_ODR_125,
    QMI8658C_ACC_ODR_62_5,
    QMI8658C_ACC_ODR_31_25,
    QMI8658C_ACC_ODR_128 = 12,
    QMI8658C_ACC_ODR_21,
    QMI8658C_ACC_ODR_11,
    QMI8658C_ACC_ODR_3,
} qmi8658c_acc_odr_t;

typedef enum {
    QMI8658C_GYRO_ODR_8000,
    QMI8658C_GYRO_ODR_4000,
    QMI8658C_GYRO_ODR_2000,
    QMI8658C_GYRO_ODR_1000,
    QMI8658C_GYRO_ODR_500,
    QMI8658C_GYRO_ODR_250,
    QMI8658C_GYRO_ODR_125,
    QMI8658C_GYRO_ODR_62_5,
    QMI8658C_GYRO_ODR_31_25,
} qmi8658c_gyro_odr_t;

typedef enum {
    QMI8658C_ACC_SCALE_2G,
    QMI8658C_ACC_SCALE_4G,
    QMI8658C_ACC_SCALE_8G,
    QMI8658C_ACC_SCALE_16G,
} qmi8658c_acc_scale_t;

typedef enum {
    QMI8658C_GYRO_SCALE_16DPS,
    QMI8658C_GYRO_SCALE_32DPS,
    QMI8658C_GYRO_SCALE_64DPS,
    QMI8658C_GYRO_SCALE_128DPS,
    QMI8658C_GYRO_SCALE_256DPS,
    QMI8658C_GYRO_SCALE_512DPS,
    QMI8658C_GYRO_SCALE_1024DPS,
    QMI8658C_GYRO_SCALE_2048DPS,
} qmi8658c_gyro_scale_t;

#define TEMPERATURE_SENSOR_RESOLUTION (1 << 8) // Telperature sensor resolution (ADC)

typedef struct
{
    float x;
    float y;
    float z;
} qmi8658c_axes_t;

typedef struct
{
    uint16_t acc_sensitivity;  // Sensitivity value for the accelerometer.
    uint8_t acc_scale;         // Scale setting for the accelerometer.
    uint16_t gyro_sensitivity; // Sensitivity value for the gyroscope.
    uint8_t gyro_scale;        // Scale setting for the gyroscope.
    uint8_t who_am_i;
    uint8_t revision;
} qmi_ctx_t;

typedef struct
{
    qmi8658c_axes_t acc;
    qmi8658c_axes_t gyro;
    float temperature;
} qmi8658c_data_t;

typedef struct
{
    qmi8658c_mode_t mode;
    qmi8658c_acc_scale_t acc_scale;
    qmi8658c_acc_odr_t acc_odr;
    qmi8658c_gyro_scale_t gyro_scale;
    qmi8658c_gyro_odr_t gyro_odr;
} qmi8658c_config_t;

esp_err_t qmi8658c_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t qmi8658c_free_desc(i2c_dev_t *dev);
esp_err_t qmi8658c_power_on(i2c_dev_t *dev);
esp_err_t qmi8658c_power_down(i2c_dev_t *dev);
esp_err_t qmi8658c_setup(i2c_dev_t *dev, qmi8658c_config_t *config);
esp_err_t qmi8658c_read_data(i2c_dev_t *dev, qmi8658c_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __QMI8658C_H__ */
/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2022 Jan Veeh <jan.veeh@motius.de>
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

/**
 * @file icm42670.h
 * @defgroup icm42670 icm42670
 * @{
 *
 * ESP-IDF driver for TDK ICM-42670-P IMU (found on ESP-RS board)
 *
 * Copyright (c) 2022 Jan Veeh (jan.veeh@motius.de)
 *
 * ISC Licensed as described in the file LICENSE
 */

#ifndef __ICM42670_H__
#define __ICM42670_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICM42670_I2C_ADDR_GND 0x68
#define ICM42670_I2C_ADDR_VCC 0x69

// Registers USER BANK 0
#define ICM42670_REG_MCLK_RDY          0x00
#define ICM42670_REG_DEVICE_CONFIG     0x01
#define ICM42670_REG_SIGNAL_PATH_RESET 0x02
#define ICM42670_REG_DRIVE_CONFIG1     0x03
#define ICM42670_REG_DRIVE_CONFIG2     0x04
#define ICM42670_REG_DRIVE_CONFIG3     0x05
#define ICM42670_REG_INT_CONFIG        0x06
#define ICM42670_REG_TEMP_DATA1        0x09
#define ICM42670_REG_TEMP_DATA0        0x0A
#define ICM42670_REG_ACCEL_DATA_X1     0x0B
#define ICM42670_REG_ACCEL_DATA_X0     0x0C
#define ICM42670_REG_ACCEL_DATA_Y1     0x0D
#define ICM42670_REG_ACCEL_DATA_Y0     0x0E
#define ICM42670_REG_ACCEL_DATA_Z1     0x0F
#define ICM42670_REG_ACCEL_DATA_Z0     0x10
#define ICM42670_REG_GYRO_DATA_X1      0x11
#define ICM42670_REG_GYRO_DATA_X0      0x12
#define ICM42670_REG_GYRO_DATA_Y1      0x13
#define ICM42670_REG_GYRO_DATA_Y0      0x14
#define ICM42670_REG_GYRO_DATA_Z1      0x15
#define ICM42670_REG_GYRO_DATA_Z0      0x16
#define ICM42670_REG_TMST_FSYNCH       0x17
#define ICM42670_REG_TMST_FSYNCL       0x18
#define ICM42670_REG_APEX_DATA4        0x1D
#define ICM42670_REG_APEX_DATA5        0x1E
#define ICM42670_REG_PWR_MGMT0         0x1F
#define ICM42670_REG_GYRO_CONFIG0      0x20
#define ICM42670_REG_ACCEL_CONFIG0     0x21
#define ICM42670_REG_TEMP_CONFIG0      0x22
#define ICM42670_REG_GYRO_CONFIG1      0x23
#define ICM42670_REG_ACCEL_CONFIG1     0x24
#define ICM42670_REG_APEX_CONFIG0      0x25
#define ICM42670_REG_APEX_CONFIG1      0x26
#define ICM42670_REG_WOM_CONFIG        0x27
#define ICM42670_REG_FIFO_CONFIG1      0x28
#define ICM42670_REG_FIFO_CONFIG2      0x29
#define ICM42670_REG_FIFO_CONFIG3      0x2A
#define ICM42670_REG_INT_SOURCE0       0x2B
#define ICM42670_REG_INT_SOURCE1       0x2C
#define ICM42670_REG_INT_SOURCE3       0x2D
#define ICM42670_REG_INT_SOURCE4       0x2E
#define ICM42670_REG_FIFO_LOST_PKT0    0x2F
#define ICM42670_REG_FIFO_LOST_PKT1    0x30
#define ICM42670_REG_APEX_DATA0        0x31
#define ICM42670_REG_APEX_DATA1        0x32
#define ICM42670_REG_APEX_DATA2        0x33
#define ICM42670_REG_APEX_DATA3        0x34
#define ICM42670_REG_INTF_CONFIG0      0x35
#define ICM42670_REG_INTF_CONFIG1      0x36
#define ICM42670_REG_INT_STATUS_DRDY   0x39
#define ICM42670_REG_INT_STATUS        0x3A
#define ICM42670_REG_INT_STATUS2       0x3B
#define ICM42670_REG_INT_STATUS3       0x3C
#define ICM42670_REG_FIFO_COUNTH       0x3D
#define ICM42670_REG_FIFO_COUNTL       0x3E
#define ICM42670_REG_FIFO_DATA         0x3F
#define ICM42670_REG_WHO_AM_I          0x75
#define ICM42670_REG_BLK_SEL_W         0x79
#define ICM42670_REG_MADDR_W           0x7A
#define ICM42670_REG_M_W               0x7B
#define ICM42670_REG_BLK_SEL_R         0x7C
#define ICM42670_REG_MADDR_R           0x7D
#define ICM42670_REG_M_R               0x7E

// MREG1 registers
#define ICM42670_REG_TMST_CONFIG1    0x00
#define ICM42670_REG_FIFO_CONFIG5    0x01
#define ICM42670_REG_FIFO_CONFIG6    0x02
#define ICM42670_REG_FSYNC_CONFIG    0x03
#define ICM42670_REG_INT_CONFIG0     0x04
#define ICM42670_REG_INT_CONFIG1     0x05
#define ICM42670_REG_SENSOR_CONFIG3  0x06
#define ICM42670_REG_ST_CONFIG       0x13
#define ICM42670_REG_SELFTEST        0x14
#define ICM42670_REG_INTF_CONFIG6    0x23
#define ICM42670_REG_INTF_CONFIG10   0x25
#define ICM42670_REG_INTF_CONFIG7    0x28
#define ICM42670_REG_OTP_CONFIG      0x2B
#define ICM42670_REG_INT_SOURCE6     0x2F
#define ICM42670_REG_INT_SOURCE7     0x30
#define ICM42670_REG_INT_SOURCE8     0x31
#define ICM42670_REG_INT_SOURCE9     0x32
#define ICM42670_REG_INT_SOURCE10    0x33
#define ICM42670_REG_APEX_CONFIG2    0x44
#define ICM42670_REG_APEX_CONFIG3    0x45
#define ICM42670_REG_APEX_CONFIG4    0x46
#define ICM42670_REG_APEX_CONFIG5    0x47
#define ICM42670_REG_APEX_CONFIG9    0x48
#define ICM42670_REG_APEX_CONFIG10   0x49
#define ICM42670_REG_APEX_CONFIG11   0x4A
#define ICM42670_REG_ACCEL_WOM_X_THR 0x4B
#define ICM42670_REG_ACCEL_WOM_Y_THR 0x4C
#define ICM42670_REG_ACCEL_WOM_Z_THR 0x4D
#define ICM42670_REG_OFFSET_USER0    0x4E
#define ICM42670_REG_OFFSET_USER1    0x4F
#define ICM42670_REG_OFFSET_USER2    0x50
#define ICM42670_REG_OFFSET_USER3    0x51
#define ICM42670_REG_OFFSET_USER4    0x52
#define ICM42670_REG_OFFSET_USER5    0x53
#define ICM42670_REG_OFFSET_USER6    0x54
#define ICM42670_REG_OFFSET_USER7    0x55
#define ICM42670_REG_OFFSET_USER8    0x56
#define ICM42670_REG_ST_STATUS1      0x63
#define ICM42670_REG_ST_STATUS2      0x64
#define ICM42670_REG_FDR_CONFIG      0x66
#define ICM42670_REG_APEX_CONFIG12   0x67

// MREG2 registers
#define ICM42670_REG_OTP_CTRL7 0x06

// MREG3 registers
#define ICM42670_REG_XA_ST_DATA 0x00
#define ICM42670_REG_YA_ST_DATA 0x01
#define ICM42670_REG_ZA_ST_DATA 0x02
#define ICM42670_REG_XG_ST_DATA 0x03
#define ICM42670_REG_YG_ST_DATA 0x04
#define ICM42670_REG_ZG_ST_DATA 0x05

/* Gyro power mode */
typedef enum {
    ICM42670_GYRO_DISABLE = 0b00,
    ICM42670_GYRO_STANDBY = 0b01,
    ICM42670_GYRO_ENABLE_LN_MODE = 0b11
} icm42670_gyro_pwr_mode_t;

/* Accelerometer power mode */
typedef enum {
    ICM42670_ACCEL_DISABLE = 0b00,
    ICM42670_ACCEL_ENABLE_LP_MODE = 0b10,
    ICM42670_ACCEL_ENABLE_LN_MODE = 0b11
} icm42670_accel_pwr_mode_t;

/* Accelerometer low power mode clock source */
typedef enum { 
    ICM42670_LP_CLK_WUO = 0, 
    ICM42670_LP_CLK_RCO = 1 
} icm42670_lp_clock_source_t;

/* Gyro FSR (full scale range) */
typedef enum {
    ICM42670_GYRO_RANGE_2000DPS = 0b00,
    ICM42670_GYRO_RANGE_1000DPS = 0b01,
    ICM42670_GYRO_RANGE_500DPS = 0b10,
    ICM42670_GYRO_RANGE_250DPS = 0b11
} icm42670_gyro_fsr_t;

/* Gyro ODR (output data rate) */
typedef enum {
    ICM42670_GYRO_ODR_12_5HZ = 0b1100,
    ICM42670_GYRO_ODR_25HZ = 0b1011,
    ICM42670_GYRO_ODR_50HZ = 0b1010,
    ICM42670_GYRO_ODR_100HZ = 0b1001,
    ICM42670_GYRO_ODR_200HZ = 0b1000,
    ICM42670_GYRO_ODR_400HZ = 0b0111,
    ICM42670_GYRO_ODR_800HZ = 0b0110,
    ICM42670_GYRO_ODR_1_6KHZ = 0b0101
} icm42670_gyro_odr_t;

/* Accelerometer FSR (full scale range) */
typedef enum {
    ICM42670_ACCEL_RANGE_16G = 0b00,
    ICM42670_ACCEL_RANGE_8G = 0b01,
    ICM42670_ACCEL_RANGE_4G = 0b10,
    ICM42670_ACCEL_RANGE_2G = 0b11
} icm42670_accel_fsr_t;

/* Accelerometer ODR (output data rate) */
typedef enum {
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

/* Temperature LPF (low pass filter) */
typedef enum {
    ICM42670_TEMP_LFP_BYPASSED = 0b000,
    ICM42670_TEMP_LFP_180HZ = 0b001,
    ICM42670_TEMP_LFP_72HZ = 0b010,
    ICM42670_TEMP_LFP_34HZ = 0b011,
    ICM42670_TEMP_LFP_16HZ = 0b100,
    ICM42670_TEMP_LFP_8HZ = 0b101,
    ICM42670_TEMP_LFP_4HZ = 0b110
} icm42670_temp_lfp_t;

/* Gyro LPF (low pass filter) */
typedef enum {
    ICM42670_GYRO_LFP_BYPASSED = 0b000,
    ICM42670_GYRO_LFP_180HZ = 0b001,
    ICM42670_GYRO_LFP_121HZ = 0b010,
    ICM42670_GYRO_LFP_73HZ = 0b011,
    ICM42670_GYRO_LFP_53HZ = 0b100,
    ICM42670_GYRO_LFP_34HZ = 0b101,
    ICM42670_GYRO_LFP_25HZ = 0b110,
    ICM42670_GYRO_LFP_16HZ = 0b111
} icm42670_gyro_lfp_t;

/* Accelerometer LPF (low pass filter) */
typedef enum {
    ICM42670_ACCEL_LFP_BYPASSED = 0b000,
    ICM42670_ACCEL_LFP_180HZ = 0b001,
    ICM42670_ACCEL_LFP_121HZ = 0b010,
    ICM42670_ACCEL_LFP_73HZ = 0b011,
    ICM42670_ACCEL_LFP_53HZ = 0b100,
    ICM42670_ACCEL_LFP_34HZ = 0b101,
    ICM42670_ACCEL_LFP_25HZ = 0b110,
    ICM42670_ACCEL_LFP_16HZ = 0b111
} icm42670_accel_lfp_t;

/* Accelerometer averaging (for low power mode) */
typedef enum {
    ICM42670_ACCEL_AVG_2X = 0b000,
    ICM42670_ACCEL_AVG_4X = 0b001,
    ICM42670_ACCEL_AVG_8X = 0b010,
    ICM42670_ACCEL_AVG_16X = 0b011,
    ICM42670_ACCEL_AVG_32X = 0b100,
    ICM42670_ACCEL_AVG_64X = 0b101
} icm42670_accel_avg_t;

/* Interrupt pin signal mode */
typedef enum {
    ICM42670_INT_MODE_PULSED = 0,
    ICM42670_INT_MODE_LATCHED = 1
} icm42670_int_mode_t;

/* Interrupt pin signal type */
typedef enum {
    ICM42670_INT_DRIVE_OPEN_DRAIN = 0,
    ICM42670_INT_DRIVE_PUSH_PULL = 1
} icm42670_int_drive_t;

/* Interrupt pin signal polarity */
typedef enum {
    ICM42670_INT_POLARITY_ACTIVE_LOW = 0,
    ICM42670_INT_POLARITY_ACTIVE_HIGH = 1
} icm42670_int_polarity_t;

/* Interrupt pin configuration */
typedef struct
{
    icm42670_int_mode_t mode;
    icm42670_int_drive_t drive;
    icm42670_int_polarity_t polarity;
} icm42670_int_config_t;

/* Interrupt source */
typedef struct
{
    bool self_test_done;
    bool fsync;
    bool pll_ready;
    bool reset_done;
    bool data_ready;
    bool fifo_threshold;
    bool fifo_full;
    bool agc_ready;
    bool i3c_error;
    bool smd;
    bool wom_z;
    bool wom_y;
    bool wom_x;
} icm42670_int_source_t;

/* Wake on Motion interrupt assertion */
typedef enum {
    ICM42670_WOM_INT_DUR_FIRST = 0b00,
    ICM42670_WOM_INT_DUR_SECOND = 0b01,
    ICM42670_WOM_INT_DUR_THIRD = 0b10,
    ICM42670_WOM_INT_DUR_FOURTH = 0b11
} icm42670_wom_int_dur_t;

/* Wake on Motion interrupt logical trigger */
typedef enum {
    ICM42670_WOM_INT_MODE_ALL_OR = 0,
    ICM42670_WOM_INT_MODE_ALL_AND = 1
} icm42670_wom_int_mode_t;

/* Wake on Motion reference sample */
typedef enum {
    ICM42670_WOM_MODE_REF_INITIAL = 0,
    ICM42670_WOM_MODE_REF_LAST = 1
} icm42670_wom_mode_t;

/* Wake on Motion configuration */
typedef struct
{
    icm42670_wom_int_dur_t trigger;
    icm42670_wom_int_mode_t logical_mode;
    icm42670_wom_mode_t reference;
    uint8_t wom_x_threshold; // 8-bit value between 0 and 1g (Resolution 1g/256=~3.9 mg)
    uint8_t wom_y_threshold; // 8-bit value between 0 and 1g (Resolution 1g/256=~3.9 mg)
    uint8_t wom_z_threshold; // 8-bit value between 0 and 1g (Resolution 1g/256=~3.9 mg)
} icm42670_wom_config_t;

/* MREG 1-3 access */
typedef enum {
    ICM42670_MREG1_RW = 0x00,
    ICM42670_MREG2_RW = 0x28,
    ICM42670_MREG3_RW = 0x50
} icm42670_mreg_number_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    // TODO: add more vars for configuration
} icm42670_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr I2C device address, `ICM42670_I2C_ADDR_...` const
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
 * @brief Set device power mode
 *
 * @param dev Device descriptor
 * @param enable_idle bool to enable idle mode
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_idle_pwr_mode(icm42670_t *dev, bool enable_idle);

/**
 * @brief Set gyro power mode
 *
 * @param dev Device descriptor
 * @param pwr_mode struct of type icm42670_gyro_pwr_mode_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_gyro_pwr_mode(icm42670_t *dev, icm42670_gyro_pwr_mode_t pwr_mode);

/**
 * @brief Set accel power mode
 *
 * @param dev Device descriptor
 * @param pwr_mode struct of type icm42670_accel_pwr_mode_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_accel_pwr_mode(icm42670_t *dev, icm42670_accel_pwr_mode_t pwr_mode);

/**
 * @brief Set clock source in LP mode
 *
 * @param dev Device descriptor
 * @param clock_source struct of type icm42670_lp_clock_source_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_low_power_clock(icm42670_t *dev, icm42670_lp_clock_source_t clock_source);

/**
 * @brief Read temperature from device
 *
 * @param dev Device descriptor
 * @param[out] temperature temperature, degree C
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_read_temperature(icm42670_t *dev, float *temperature);

/**
 * @brief Read 16-bit raw data registers (accelerometer and gyro values)
 *
 * @param dev Device descriptor
 * @param data_register data register to read from
 * @param[out] data accel or gyro data
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_read_raw_data(icm42670_t *dev, uint8_t data_register, int16_t *data);

/**
 * @brief Performs a soft-reset
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_reset(icm42670_t *dev);

/**
 * @brief Wipes the FIFO
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_flush_fifo(icm42670_t *dev);

/**
 * @brief Set the measurement FSR (Full Scale Range) of the gyro
 *
 * @param dev Device descriptor
 * @param range struct of type icm42670_gyro_fsr_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_gyro_fsr(icm42670_t *dev, icm42670_gyro_fsr_t range);

/**
 * @brief Set the measurement ODR (Output Data Rate) of the gyro
 *
 * @param dev Device descriptor
 * @param odr struct of type icm42670_gyro_odr_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_gyro_odr(icm42670_t *dev, icm42670_gyro_odr_t odr);

/**
 * @brief Set the measurement FSR (Full Scale Range) of the accelerometer
 *
 * @param dev Device descriptor
 * @param range struct of type icm42670_accel_fsr_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_accel_fsr(icm42670_t *dev, icm42670_accel_fsr_t range);

/**
 * @brief Set the measurement ODR (Output Data Rate) of the accelerometer
 *
 * @param dev Device descriptor
 * @param odr struct of type icm42670_accel_odr_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_accel_odr(icm42670_t *dev, icm42670_accel_odr_t odr);

/**
 * @brief Set the digital Low-Pass-Filter (LPF) of the temperature sensor
 *
 * @param dev Device descriptor
 * @param lpf_bw struct of type icm42670_temp_lfp_t (bandwidth)
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_temp_lpf(icm42670_t *dev, icm42670_temp_lfp_t lpf_bw);

/**
 * @brief Set the digital Low-Pass-Filter (LPF) of the gyro
 *
 * @param dev Device descriptor
 * @param lpf_bw struct of type icm42670_gyro_lfp_t (bandwidth)
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_gyro_lpf(icm42670_t *dev, icm42670_gyro_lfp_t lpf_bw);

/**
 * @brief Set the digital Low-Pass-Filter (LPF) of the accelerometer
 *
 * @param dev Device descriptor
 * @param lpf_bw struct of type icm42670_accel_lfp_t (bandwidth)
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_accel_lpf(icm42670_t *dev, icm42670_accel_lfp_t lpf_bw);

/**
 * @brief Set the averaging filter of the accelerometer (ONLY IN LOW POWER MODE (LPM))
 *        This field can not be changed, when accel sensor is in LPM!
 *
 * @param dev Device descriptor
 * @param avg struct of type icm42670_accel_avg_t (averaging)
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_accel_avg(icm42670_t *dev, icm42670_accel_avg_t avg);

/**
 * @brief Configures the behaviour of an interrupt pin
 *
 * @param dev Device descriptor
 * @param int_pin interrupt pin (1 or 2)
 * @param config struct of type icm42670_int_config_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_config_int_pin(icm42670_t *dev, uint8_t int_pin, icm42670_int_config_t config);

/**
 * @brief Configures the sources for an interrupt
 *
 * @param dev Device descriptor
 * @param int_pin interrupt pin (1 or 2)
 * @param sources struct of type icm42670_int_source_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_set_int_sources(icm42670_t *dev, uint8_t int_pin, icm42670_int_source_t sources);

/**
 * @brief Configures the Wake on Motion (WoM) behaviour
 *        WoM can only be configured if WoM is not enabled
 *
 * @param dev Device descriptor
 * @param config struct of type icm42670_wom_config_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_config_wom(icm42670_t *dev, icm42670_wom_config_t config);

/**
 * @brief Enable or Disable Wake on Motion (WoM)
 *
 * @param dev Device descriptor
 * @param enable true to enable, false to disable
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_enable_wom(icm42670_t *dev, bool enable);

/**
 * @brief Get the status of the internal clock
 *
 * @param dev Device descriptor
 * @param mclk_rdy true if internal clock is running
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_get_mclk_rdy(icm42670_t *dev, bool *mclk_rdy);

/**
 * @brief Get the output data rate (ODR) of the accel
 *
 * @param dev Device descriptor
 * @param odr pointer to icm42670_accel_odr_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_get_accel_odr(icm42670_t *dev, icm42670_accel_odr_t *odr);

/**
 * @brief Get the status of the accel averaging
 *
 * @param dev Device descriptor
 * @param avg pointer to icm42670_accel_avg_t
 * @return `ESP_OK` on success
 */
esp_err_t icm42670_get_accel_avg(icm42670_t *dev, icm42670_accel_avg_t *avg);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __ICM42670_H__

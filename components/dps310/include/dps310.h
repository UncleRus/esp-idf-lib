/*
 * Copyright (c) 2022 Tomoyuki Sakurai <y@trombik.org>
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
 *
 * Sponsored by @beriberikix <https://github.com/beriberikix>
 */

/**
 * @file dps310.h
 * @defgroup dps310 dps310
 * @{
 *
 * ESP-IDF driver for DPS310 barometric pressure sensor. Sponsored by beriberikix.
 *
 * DPS310 supports I2C and SPI (3-wires and 4-wires) as digital interface. The
 * driver currently supports:
 *
 * * I2C
 *
 * The driver currently does not support:
 *
 * * SPI
 * * read measurements by interrupt
 * * multi-master I2C configuration
 *
 * Note that the unit of pressure in this driver is pascal (Pa), not
 * hectopascals (hPa).
 *
 * Note that the unit of altitude in this driver is meter.
 *
 */
#if !defined(__DPS310_H__)
#define __DPS310_H__

/* standard headers */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* esp-idf headers */
#include <esp_err.h>

/* esp-idf-lib headers */
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DPS310_I2C_ADDRESS_0  0x76 //!< I2C address when SDO pin is low
#define DPS310_I2C_ADDRESS_1  0x77 //!< I2C address when SDO pin is high

#define DPS310_PRODUCT_ID   0x01 //!< Product ID
#define DPS310_REVISION_ID  0x00 //!< Revision ID

#define DPS310_AVERAGE_SEA_LEVEL_PRESSURE_hPa   (1013.25) //!< Average sea-level pressure in hPa
#define DPS310_AVERAGE_SEA_LEVEL_PRESSURE_Pa    (DPS310_AVERAGE_SEA_LEVEL_PRESSURE_hPa * 10) //!< Average sea-level pressure in Pa

/**
 * Mode of DPS310 module operation. See 4.1 Operating Modes.
 */
typedef enum {
    DPS310_MODE_STANDBY = 0b000,  //!< Standby mode
    DPS310_MODE_COMMAND_PRESSURE = 0b001, //!<  Command mode, pressure measurement
    DPS310_MODE_COMMAND_TEMPERATURE = 0b010, //!<  Command mode, temperature measurement
    DPS310_MODE_BACKGROUND_PRESSURE = 0b101, //!<  Background mode, continuous pressure measurement
    DPS310_MODE_BACKGROUND_TEMPERATURE = 0b110, //!<  Background mode, continuous temperature measurement
    DPS310_MODE_BACKGROUND_ALL = 0b111, //!<  Background mode, continuous pressure and temperature measurement
} dps310_mode_t;

/**
 * Pressure measurement rate.
 */
typedef enum {
    DPS310_PM_RATE_1   = 0b000, //!<   1 measurements / sec
    DPS310_PM_RATE_2   = 0b001, //!<   2 measurements / sec
    DPS310_PM_RATE_4   = 0b010, //!<   4 measurements / sec
    DPS310_PM_RATE_8   = 0b011, //!<   8 measurements / sec
    DPS310_PM_RATE_16  = 0b100, //!<  16 measurements / sec
    DPS310_PM_RATE_32  = 0b101, //!<  32 measurements / sec
    DPS310_PM_RATE_64  = 0b110, //!<  64 measurements / sec
    DPS310_PM_RATE_128 = 0b111, //!< 128 measurements / sec
} dps310_pm_rate_t;

/**
 * Pressure resolution, or oversampling rate.
 */
typedef enum {
    DPS310_PM_PRC_1   = 0b000, //!<   Single (Low Precision)
    DPS310_PM_PRC_2   = 0b001, //!<   2 times (Low Power).
    DPS310_PM_PRC_4   = 0b010, //!<   4 times
    DPS310_PM_PRC_8   = 0b011, //!<   8 times
    DPS310_PM_PRC_16  = 0b100, //!<  16 times (Standard)
    DPS310_PM_PRC_32  = 0b101, //!<  32 times
    DPS310_PM_PRC_64  = 0b110, //!<  64 times (High Precision)
    DPS310_PM_PRC_128 = 0b111, //!< 128 times
} dps310_pm_oversampling_t;

/**
 * Temperature measurement source. Used for temperature measurement and
 * temperature coefficients.
 */

typedef enum {
    DPS310_TMP_SRC_INTERNAL = 0,    //!< Internal sensor (in ASIC)
    DPS310_TMP_SRC_EXTERNAL = 1,    //!< External sensor (in pressure sensor MEMS element)
} dps310_tmp_src_ext_t;

/**
 * Temperature measurement rate.
 */
typedef enum {
    DPS310_TMP_RATE_1   = 0b000, //!<   1 measurements / sec
    DPS310_TMP_RATE_2   = 0b001, //!<   2 measurements / sec
    DPS310_TMP_RATE_4   = 0b010, //!<   4 measurements / sec
    DPS310_TMP_RATE_8   = 0b011, //!<   8 measurements / sec
    DPS310_TMP_RATE_16  = 0b100, //!<  16 measurements / sec
    DPS310_TMP_RATE_32  = 0b101, //!<  32 measurements / sec
    DPS310_TMP_RATE_64  = 0b110, //!<  64 measurements / sec
    DPS310_TMP_RATE_128 = 0b111, //!< 128 measurements / sec
} dps310_tmp_rate_t;

/**
 * Pressure resolution, or oversampling rate.
 */
typedef enum {
    DPS310_TMP_PRC_1   = 0b000, //!<   Single (Low Precision)
    DPS310_TMP_PRC_2   = 0b001, //!<   2 times (Low Power).
    DPS310_TMP_PRC_4   = 0b010, //!<   4 times
    DPS310_TMP_PRC_8   = 0b011, //!<   8 times
    DPS310_TMP_PRC_16  = 0b100, //!<  16 times (Standard)
    DPS310_TMP_PRC_32  = 0b101, //!<  32 times
    DPS310_TMP_PRC_64  = 0b110, //!<  64 times (High Precision)
    DPS310_TMP_PRC_128 = 0b111, //!< 128 times
} dps310_tmp_oversampling_t;

/**
 * Interupt (on SDO pin) active level.
 */
typedef enum {
    DPS310_INT_HL_ACTIVE_LOW = 0,  //!< Active low
    DPS310_INT_HL_ACTIVE_HIGH = 1, //!< Active high
} dps310_int_hl_active_level_t;

/**
 * Mode of interupt when the FIFO is full.
 */
typedef enum {
    DPS310_INT_FIFO_DISABLE = 0, //!< Disable interrupt when the FIFO is full
    DPS310_INT_FIFO_ENABLE  = 1, //!< Enable interrupt when the FIFO is full
} dps310_int_fifo_mode_t;

/**
 * Mode of interupt when a temperature measurement is ready
 */
typedef enum {
    DPS310_INT_TMP_DISABLE  = 0, //!< Disable interrupt when a temperature measurement is ready
    DPS310_INT_TMP_ENABLE   = 1, //!< Enable interrupt when a temperature measurement is ready
} dps310_int_tmp_mode_t;

/**
 * Mode of interupt when a pressure measurement is ready
 */
typedef enum {
    DPS310_INT_PRS_DISABLE  = 0, //!< Disable interrupt when a pressure measurement is ready
    DPS310_INT_PRS_ENABLE   = 1, //!< Enable interrupt when a pressure measurement is ready
} dps310_int_prs_mode_t;

/**
 * Mode of temperature result bit-shift.
 */
typedef enum {
    DPS310_T_SHIFT_DISABLE  = 0, //!< No shift.
    DPS310_T_SHIFT_ENABLE   = 1, //!< Shift result right in data register.
                                 //   Must be set to '1' when the oversampling
                                 //   rate is >8 times.
} dps310_t_shift_mode_t;

/**
 * Mode of pressure result bit-shift.
 */
typedef enum {
    DPS310_P_SHIFT_DISABLE  = 0, //!< No shift.
    DPS310_P_SHIFT_ENABLE   = 1, //!< Shift result right in data register.
                                 //   Must be set to '1' when the oversampling
                                 //   rate is >8 times.
} dps310_p_shift_mode_t;

/**
 * Mode of FIFO.
 */
typedef enum {
    DPS310_FIFO_DISABLE = 0, //!< Disable FIFO.
    DPS310_FIFO_ENABLE  = 1, //!< Enable FIFO.
} dps310_fifo_en_mode_t;

/**
 * SPI mode.
 */
typedef enum {
    DPS310_SPI_MODE_4WIRE = 0, //!< SPI 4-wires
    DPS310_SPI_MODE_3WIRE = 1, //!< SPI 3-wires
} dps310_spi_mode_t;

/**
 * Type of measurement result in FIFO.
 *
 * When the type is DPS310_MEASUREMENT_EMPTY, the result is always zero.
 * Otherwise, the result is the compensated value of each type.
 */
typedef enum {
    DPS310_MEASUREMENT_TEMPERATURE = 0, //!< Temperature
    DPS310_MEASUREMENT_PRESSURE,        //!< Pressure
    DPS310_MEASUREMENT_EMPTY,           //!< Empty, no measurement available
} dps310_fifo_measurement_type_t;

typedef struct {
    dps310_fifo_measurement_type_t type;
    float result;
} dps310_fifo_measurement_t;

/**
 * Configuration parameters for DPS310.
 */
typedef struct {
    dps310_pm_rate_t pm_rate;
    dps310_pm_oversampling_t pm_oversampling;
    dps310_tmp_rate_t tmp_rate;
    dps310_tmp_oversampling_t tmp_oversampling;
    dps310_tmp_src_ext_t tmp_src;
    dps310_tmp_src_ext_t tmp_coef;
    dps310_int_fifo_mode_t int_fifo_mode;
    dps310_int_tmp_mode_t int_tmp_mode;
    dps310_int_prs_mode_t int_prs_mode;
    dps310_t_shift_mode_t t_shift_mode;
    dps310_p_shift_mode_t p_shift_mode;
    dps310_fifo_en_mode_t fifo_en_mode;
    dps310_spi_mode_t spi_mode;

} dps310_config_t;

/**
 * A macro to set default dps310_config_t.
 */

#define DPS310_CONFIG_DEFAULT() { \
    .pm_rate = DPS310_PM_RATE_1, \
    .pm_oversampling = DPS310_PM_PRC_16, \
    .tmp_rate = DPS310_TMP_RATE_1, \
    .tmp_oversampling = DPS310_TMP_PRC_16, \
    .tmp_src = DPS310_TMP_SRC_EXTERNAL, \
    .tmp_coef = DPS310_TMP_SRC_EXTERNAL, \
    .int_fifo_mode = DPS310_INT_FIFO_DISABLE, \
    .int_tmp_mode = DPS310_INT_TMP_DISABLE, \
    .int_prs_mode = DPS310_INT_PRS_DISABLE, \
    .t_shift_mode = DPS310_T_SHIFT_ENABLE, \
    .p_shift_mode = DPS310_P_SHIFT_ENABLE, \
    .fifo_en_mode = DPS310_FIFO_DISABLE, \
    .spi_mode = DPS310_SPI_MODE_4WIRE, \
}

/**
 * Calibration Coefficients (COEF).
 */
typedef struct {
    int32_t c0;
    int32_t c1;
    int32_t c00;
    int32_t c10;
    int32_t c01;
    int32_t c11;
    int32_t c20;
    int32_t c21;
    int32_t c30;
} dps310_coef_t;

/**
 * Device descriptor.
 */
typedef struct {
    i2c_dev_t i2c_dev;          //!< I2C device descriptor
    uint8_t prod_id;            //!< Product ID
    uint8_t prod_rev;           //!< Product revision
    uint8_t t_rate;             //!< latest P_rate
    uint8_t p_rate;             //!< latest T_rate
    int32_t t_raw;              //!< latest T_raw
    dps310_coef_t coef;         //!< coefficients
    float offset;               //!< offset in meter
    float pressure_s;           //!< calculated pressure at sea-level
} dps310_t;

/**
 * DPS310 registers
 */

/* 7 Register Map */
#define DPS310_REG_PRS_B2       0x00
#define DPS310_REG_PRS_B1       0x01
#define DPS310_REG_PRS_B0       0x02
#define DPS310_REG_TMP_B2       0x03
#define DPS310_REG_TMP_B1       0x04
#define DPS310_REG_TMP_B0       0x05
#define DPS310_REG_PRS_CFG      0x06
#define DPS310_REG_TMP_CFG      0x07
#define DPS310_REG_MEAS_CFG     0x08
#define DPS310_REG_CFG_REG      0x09
#define DPS310_REG_INT_STS      0x0a
#define DPS310_REG_FIFO_STS     0x0b
#define DPS310_REG_RESET        0x0c
#define DPS310_REG_ID           0x0d
#define DPS310_REG_COEF         0x10
#define DPS310_REG_COEF_LEN     (18)
#define DPS310_REG_COEF_SRCE    0x28

/* various masks */
#define DPS310_REG_ID_REV_MASK              (0x0f)
#define DPS310_REG_ID_PROD_MASK             (0xf0)
#define DPS310_REG_RESET_FIFO_FLUSH_MASK    (1 << 7)
#define DPS310_REG_RESET_SOFT_RST_MASK      (0x0f)
#define DPS310_REG_PRS_CFG_PM_RATE_MASK     (0b111 << 4)
#define DPS310_REG_PRS_CFG_TMP_RATE_MASK    (0b111 << 4)
#define DPS310_REG_PRS_CFG_PM_PRC_MASK      (0b1111)
#define DPS310_REG_PRS_CFG_TMP_EXT_MASK     (1 << 7)
#define DPS310_REG_TMP_CFG_TMP_PRC_MASK     (0b1111)
#define DPS310_REG_CFG_REG_INT_HL_MASK      (1 << 7)
#define DPS310_REG_CFG_REG_INT_FIFO_MASK    (1 << 6)
#define DPS310_REG_CFG_REG_INT_TMP_MASK     (1 << 5)
#define DPS310_REG_CFG_REG_INT_PRS_MASK     (1 << 4)
#define DPS310_REG_CFG_REG_T_SHIFT_MASK     (1 << 3)
#define DPS310_REG_CFG_REG_P_SHIFT_MASK     (1 << 2)
#define DPS310_REG_CFG_REG_FIFO_EN_MASK     (1 << 1)
#define DPS310_REG_CFG_REG_SPI_MODE_MASK    (1 << 0)
#define DPS310_REG_MEAS_CFG_COEF_RDY_MASK   (1 << 7)
#define DPS310_REG_MEAS_CFG_SENSOR_RDY_MASK (1 << 6)
#define DPS310_REG_MEAS_CFG_TMP_RDY_MASK    (1 << 5)
#define DPS310_REG_MEAS_CFG_PRS_RDY_MASK    (1 << 4)
#define DPS310_REG_MEAS_CFG_MEAS_CTRL_MASK  (0b111)
#define DPS310_REG_COEF_SRCE_MASK           (1 << 7)
#define DPS310_REG_FIFO_STS_FIFO_EMPTY_MASK (1)
#define DPS310_REG_FIFO_STS_FIFO_FULL_MASK  (1 << 1)

/* See 3.6 Timing Characteristics */
#define DPS310_I2C_FREQ_MAX_HZ  (3400000)  // Max 3.4 MHz
                                           //
/* I2C master driver does not support higher than 1MHz
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#_CPPv4N12i2c_config_t9clk_speedE
 */
#define DPS310_I2C_FREQ_MAX_ESP_IDF_HZ  (1000000)
#define DPS310_SPI_FREQ_MAX_HZ  (10000000) // Max 10 MHz
                                           //
/* See 4.3 Start-up sequence
 *
 * XXX the datasheet is ambiguous in the start-up sequence. Trim_complete is
 * mentioned in nowhere. the DPS310-Pressure-Sensor by Infineon uses 50 ms.
 * don't know what the "40ms" in the chart means. to be safe, use the sum of
 * all the numbers in the chart.
 */
#define DPS310_TRIM_READY_DELAY_MS  (3) // 2.5 ms
#define DPS310_SENSOR_READY_DELAY_MS  (12)
#define DPS310_COEFFICIENTS_READY_DELAY_MS  (40)
#define DPS310_STARTUP_DELAY_MS (DPS310_TRIM_READY_DELAY_MS + DPS310_SENSOR_READY_DELAY_MS + DPS310_COEFFICIENTS_READY_DELAY_MS)

#define DPS310_PROD_ID  0x01

/* temperature and pressure use three resisters for 24 bits values. */
#define DPS310_REG_SENSOR_VALUE_LEN (3)

/* 4.8 FIFO Operation */
#define DPS310_REG_FIFO     DPS310_REG_PRS_B2   //! Resister address of FIFO.
#define DPS310_FIFO_EMPTY   (0xff800000)        //! the value of two's complement in the resisters when no measurement is in the FIFO.

/* See 8.9 Soft Reset and FIFO flush (RESET) */
#define DPS310_FIFO_FLUSH_VALUE (1 << 7)
#define DPS310_SOFT_RST_VALUE   (0b1001)

/**
 * @brief Initialize device descriptor
 *
 * @param[out] dev      The device descriptor.
 * @param[in]  addr     DPS310's I2C address
 * @param[in]  port     I2C port number to use.
 *                 See available I2C port at:
 *                 https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#_CPPv410i2c_port_t
 * @param[in]  sda_gpio GPIO pin for SDA
 * @param[in]  scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t dps310_init_desc(dps310_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free the device descriptor.
 *
 * The device descriptor must NOT be NULL. `dps310_free_desc()` does not
 * `free()` the device descriptor on error.
 *
 * @param[out] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t dps310_free_desc(dps310_t *dev);

/**
 * @brief Initialize DPS310 module
 *
 * The function does the followings:
 *
 * - read the DPS310_REG_ID, and identify the product ID. Return ESP_FAIL if
 *   the product ID does not match expected product ID.
 * - reset the chip
 * - perform a quirk
 *
 * @param[in] dev    Device descriptor
 * @param[in] config Configuration
 * @return `ESP_OK` on success
 */
esp_err_t dps310_init(dps310_t *dev, dps310_config_t *config);

/**
 * @brief Reset the device.
 *
 * Perform "soft reset" and ensure the chip is fully functional by delaying
 * `DPS310_STARTUP_DELAY_MS`.
 *
 * @param[in] dev The device descriptor
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `config` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_reset(dps310_t *dev);

/**
 * @brief Get pressure measurement rate.
 *
 * @param[in] dev The device descriptor
 * @param[out] value the value in the resister
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` and/or
 * `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_rate_p(dps310_t *dev, dps310_pm_rate_t *value);

/**
 * @brief Set pressure measurement rate.
 *
 * @param[in] dev The device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `config` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_rate_p(dps310_t *dev, dps310_pm_rate_t value);

/**
 * @brief Get temperature measurement rate.
 *
 * @param[in] dev The device descriptor
 * @param[out] value the value in the resister
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` and/or
 * `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_rate_t(dps310_t *dev, dps310_tmp_rate_t *value);

/**
 * @brief Set temperature measurement rate.
 *
 * @param[in] dev The device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `config` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_rate_t(dps310_t *dev, dps310_tmp_rate_t value);

/**
 * @brief Get pressure oversampling rate.
 *
 * @param[in] dev The device descriptor
 * @param[out] value the value in the resister
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` and/or
 * `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_oversampling_p(dps310_t *dev, dps310_pm_oversampling_t *value);

/**
 * @brief Set pressure oversampling rate.
 *
 * @param[in] dev The device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `config` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_oversampling_p(dps310_t *dev, dps310_pm_oversampling_t value);

/**
 * @brief Get temperature oversampling rate.
 *
 * @param[in] dev The device descriptor
 * @param[out] value the value in the resister
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` and/or
 * `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_oversampling_t(dps310_t *dev, dps310_tmp_oversampling_t *value);

/**
 * @brief Set temperature oversampling rate.
 *
 * @param[in] dev The device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `config` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_oversampling_t(dps310_t *dev, dps310_tmp_oversampling_t value);

/**
 * @brief Get temperature measurement source.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value the value in the resister.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_tmp_ext(dps310_t *dev, dps310_tmp_src_ext_t *value);

/**
 * @brief Set temperature measurement source.
 *
 * @param[in] dev The device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_tmp_ext(dps310_t *dev, dps310_tmp_src_ext_t value);

/**
 * @brief Set temperature coefficient source.
 *
 * @param[in] dev The device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_tmp_coef_ext(dps310_t *dev, dps310_tmp_src_ext_t value);

/**
 * @brief Get interrupt active level.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value the value in the resister.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_int_hl(dps310_t *dev, dps310_int_hl_active_level_t *value);

/**
 * @brief Set interrupt active level.
 *
 * @param[in] dev The device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_int_hl(dps310_t *dev, dps310_int_hl_active_level_t value);

/**
 * @brief Get the status of FIFO interrupt.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value Current configuration of INT_FIFO.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_int_fifo(dps310_t *dev, dps310_int_fifo_mode_t *value);

/**
 * @brief Set the status of FIFO interrupt.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_int_fifo(dps310_t *dev, dps310_int_fifo_mode_t value);

/**
 * @brief Get the status of temperature interrupt.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value Current configuration of INT_TMP.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_int_tmp(dps310_t *dev, dps310_int_tmp_mode_t *value);

/**
 * @brief Set the status of temperature interrupt.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_int_tmp(dps310_t *dev, dps310_int_tmp_mode_t value);

/**
 * @brief Get the status of pressure interrupt.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value Current configuration of INT_PRS.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_int_prs(dps310_t *dev, dps310_int_prs_mode_t *value);

/**
 * @brief Set the status of pressure interrupt.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_int_prs(dps310_t *dev, dps310_int_prs_mode_t value);

/**
 * @brief Get the status of temperature result bit-shift.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value Current configuration of T_SHIFT.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_t_shift(dps310_t *dev, dps310_t_shift_mode_t *value);

/**
 * @brief Set the status of temperature result bit-shift.
 *
 * Must be set to DPS310_T_SHIFT_ENABLE when the oversampling rate is >8 times.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_t_shift(dps310_t *dev, dps310_t_shift_mode_t value);

/**
 * @brief Get the status of pressure result bit-shift.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value Current configuration of T_SHIFT.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_p_shift(dps310_t *dev, dps310_p_shift_mode_t *value);

/**
 * @brief Set the status of pressure result bit-shift.
 *
 * Must be set to DPS310_P_SHIFT_ENABLE when the oversampling rate is >8 times.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_p_shift(dps310_t *dev, dps310_p_shift_mode_t value);

/**
 * @brief Get the status of FIFO.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value Current configuration of FIFO_EN.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_fifo_en(dps310_t *dev, dps310_fifo_en_mode_t *value);

/**
 * @brief Set the status of FIFO.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_fifo_en(dps310_t *dev, dps310_fifo_en_mode_t value);

/**
 * @brief Get the mode of SPI.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value Current configuration of SPI_MODE.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_spi_mode(dps310_t *dev, dps310_spi_mode_t *value);

/**
 * @brief Set the mode of SPI.
 *
 * @param[in] dev The device descriptor.
 * @param[out] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_spi_mode(dps310_t *dev, dps310_spi_mode_t value);

/**
 * @brief Get Calibration Coefficients (COEF), update COEF in the device
 * descriptor.
 *
 * @param[in] dev The device descriptor.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_coef(dps310_t *dev);

/**
 * @brief Get operating mode.
 *
 * @param[in] dev The device descriptor.
 * @param[out] mode The operating mode.
 */
esp_err_t dps310_get_mode(dps310_t *dev, dps310_mode_t *mode);

/**
 * @brief Set operating mode.
 *
 * @param[in] dev The device descriptor.
 * @param[in] mode The operating mode.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_mode(dps310_t *dev, dps310_mode_t mode);

/**
 * @brief Flush FIFO.
 *
 * @param[in] dev The device descriptor.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_flush_fifo(dps310_t *dev);

/**
 * @brief Enable or disable FIFO.
 *
 * The function performs flush (`dps310_flush_fifo()`) before disabling FIFO.
 *
 * @param[in] dev The device descriptor.
 * @param[in] enable Enable FIFO when true, disable FIFO when false.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_enable_fifo(dps310_t *dev, bool enable);

/**
 * @brief Read the raw sensor value from resisters.
 *
 * The real raw value is 2's complement. The function internally converts the
 * value from the 2's complement to uint32_t number.
 *
 * @param[in] dev The device descriptor.
 * @param[in] reg Either `DPS310_REG_TMP_B2` or `DPS310_REG_PRS_B2`.
 * @param[out] value The raw value in the three resisters.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_read_raw(dps310_t *dev, uint8_t reg, int32_t *value);

/**
 * @brief Read compensated pressure value.
 *
 * @param[in] dev The device descriptor.
 * @param[out] pressure Compensated pressure value in Pascal (not hPa).
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_read_pressure(dps310_t *dev, float *pressure);

/**
 * @brief Read compensated temperature value after waiting for PRES_RDY bit.
 *
 * @param[in] dev The device descriptor.
 * @param[in] delay_ms Time in microseconds to wait when the value is not ready.
 * @param[in] max_attempt Number of attempt to read.
 * @param[out] pressure Compensated pressure value in Pascal (not hPa).
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL. ESP_ERR_TIMEOUT when failed to read the measurement within max_attempt, or other errors when I2C communication fails.
 */
esp_err_t dps310_read_pressure_wait(dps310_t *dev, uint16_t delay_ms, uint8_t max_attempt, float *pressure);

/**
 * @brief Read compensated temperature value.
 *
 * @param[in] dev The device descriptor.
 * @param[out] temperature Compensated temperature value.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_read_temp(dps310_t *dev, float *temperature);

/**
 * @brief Read compensated temperature value after waiting for TMP_RDY bit.
 *
 * @param[in] dev The device descriptor.
 * @param[in] delay_ms Time in microseconds to wait when the value is not ready.
 * @param[in] max_attempt Number of attempt to read.
 * @param[out] temperature Compensated temperature value.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL. ESP_ERR_TIMEOUT when failed to read the measurement within max_attempt, or other errors when I2C communication fails.
 */
esp_err_t dps310_read_temp_wait(dps310_t *dev, uint16_t delay_ms, uint8_t max_attempt, float *temperature);

/**
 * @brief Test if a single bit in a resister is set.
 *
 * @param[in] dev The device descriptor.
 * @param[in] reg The resister
 * @param[in] mask bit mask to test
 * @param[out] ready true when the bit is set, false when the bit is cleared.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_is_ready_for(dps310_t *dev, uint8_t reg, uint8_t mask, bool *ready);

/**
 * @brief Test COEF_RDY in MEAS_CFG resister is set.
 *
 * @param[in] dev The device descriptor.
 * @param[out] ready true when the bit is set, false when the bit is cleared.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_is_ready_for_coef(dps310_t *dev, bool *ready);

/**
 * @brief Test SENSOR_RDY in MEAS_CFG resister is set.
 *
 * @param[in] dev The device descriptor.
 * @param[out] ready true when the bit is set, false when the bit is cleared.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_is_ready_for_sensor(dps310_t *dev, bool *ready);

/**
 * @brief Test TMP_RDY in MEAS_CFG resister is set.
 *
 * @param[in] dev The device descriptor.
 * @param[out] ready true when the bit is set, false when the bit is cleared.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_is_ready_for_temp(dps310_t *dev, bool *ready);

/**
 * @brief Test PRS_RDY in MEAS_CFG resister is set.
 *
 * @param[in] dev The device descriptor.
 * @param[out] ready true when the bit is set, false when the bit is cleared.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_is_ready_for_pressure(dps310_t *dev, bool *ready);

/**
 * @brief Reset undocumented internal resisters.
 *
 * The function is supposed to fix an issue in the sensor by writing magic
 * values to magic resisters. However, the issue is not documented. The
 * latest data sheet does not mention the issue, nor an errata.
 *
 * After issuing magic commands, the function re-reads COEF and temperature
 * once so that the subsequent pressure reads return compensated values with
 * internal cached parameters.
 *
 * See:
 * https://github.com/Infineon/DPS310-Pressure-Sensor#temperature-measurement-issue
 * https://github.com/Infineon/DPS310-Pressure-Sensor/blob/3edb0e58dfd7691491ae8d7f6a86277b001ad93f/src/DpsClass.cpp#L442-L461
 * https://github.com/Infineon/DPS310-Pressure-Sensor/blob/ed02f803fc780cbcab54ed8b35dd3d718f2ebbda/src/Dps310.cpp#L84-L86
 * https://github.com/Infineon/DPS310-Pressure-Sensor/issues/15#issuecomment-475394536
 *
 * @param[in] dev The device descriptor.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_quirk(dps310_t *dev);

/**
 * @brief See if FIFO is empty.
 *
 * @param[in] dev The device descriptor.
 * @param[out] result The result. true if empty, false otherwise.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_is_fifo_empty(dps310_t *dev, bool *result);

/**
 * @brief Read measurement result from FIFO.
 *
 * @param[in] dev The device descriptor.
 * @param[out] measurement Measured value.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_read_fifo(dps310_t *dev, dps310_fifo_measurement_t *measurement);

/**
 * @brief Start background measurement.
 *
 * This function is a syntax-sugar of `dps310_set_mode()` just for readbility
 * and for an emphasis on a fact that measurement starts immediately after
 * this.
 *
 * @param[in] dev The device descriptor.
 * @param[in] mode The mode of background measurement.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_backgorund_start(dps310_t *dev, dps310_mode_t mode);

/**
 * @brief Stop background measurement.
 *
 * This function is a syntax-sugar of `dps310_set_mode()`.
 *
 * @param[in] dev The device descriptor.
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_backgorund_stop(dps310_t *dev);

/**
 * @brief Calibrate altitude offset from the altitude of the device.
 *
 * Call this function before dps310_read_altitude() for higher accuracy.
 *
 * By default, the driver calculates altitude using average sea-level
 * pressure. This function updates internal offset of altitude by reading
 * pressure from the sensor, and given altitude. There are public web services
 * that provide altitude at a specific location, such as Google Earth.
 *
 * The function attempts to keep original oversampling rates during
 * calibration. When it fails to do so due to errors, the oversampling rates
 * might be different.
 *
 * @param[in] dev The device descriptor.
 * @param[in] altitude_real Real (known) altitude.
 */
esp_err_t dps310_calibrate_altitude(dps310_t *dev, float altitude_real);

/**
 * @brief Calculate altitude from pressure.
 *
 * Calculates altitude from pressure given. Call dps310_calibrate_altitude()
 * before this function for higher accuracy. The function adds the offset to
 * calculated altitude.
 *
 * @param[in] dev The device descriptor.
 * @param[in] pressure The pressure.
 * @param[out] altitude The calicurated altitude.
 */
esp_err_t dps310_calc_altitude(dps310_t *dev, float pressure, float *altitude);

/**
 * @brief Read pressure from the sensor, calculates altitude.
 *
 * Make sure that pressure measurement value is available.
 *
 * @param[in] dev The device descriptor.
 * @param[out] altitude The calculated altitude.
 */
esp_err_t dps310_read_altitude(dps310_t *dev, float *altitude);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __DPS310_H__

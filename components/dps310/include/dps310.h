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
 * XXX time between command mode?
 */

/**
 * @file dps310.h
 * @defgroup dps310 dps310
 * @{
 *
 * ESP-IDF driver for DPS310 barometric pressure sensor. Sponserd by @beriberikix.
 *
 * DPS310 supports I2C and SPI (3-wires and 4-wires) as digital interface. The
 * driver currently supports:
 *
 * * I2C
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

/**
 * Mode of DPS310 module operation. See 4.1 Operating Modes.
 */
typedef enum {
    DPS310_MODE_STANDBY = 0b000,  //!< Standby mode
    DPS310_MODE_COMMAND_PRESSURE = 0b001, //!<  Command mode, pressure measurement
    DPS310_MODE_COMMAND_TEMPERATURE = 0b010, //!<  Command mode, temperature measurement
    DPS310_MODE_BACKGROUND_PRESSURE = 0b101, //!<  Background mode, continous pressure measurement
    DPS310_MODE_BACKGROUND_TEMPERATURE = 0b110, //!<  Background mode, continous temperature measurement
    DPS310_MODE_BACKGROUND_ALL = 0b110, //!<  Background mode, continous pressure and temperature measurement
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
} dps310_pm_prc_t;

/**
 * Temperature measurement source. Used for temperature measurement and
 * temperature coefficients.
 */

typedef enum {
    DPS310_TMP_SRC_INTERNAL = 0,    //!< Internal sensor (in ASIC)
    DPS310_TMP_SRC_EXTERNAL = 1,         //!< External sensor (in pressure sensor MEMS element)
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
} dps310_tmp_prc_t;

/**
 * Interupt (on SDO pin) active level.
 */
typedef enum {
    DPS310_INT_HL_ACTIVE_LOW = 0,  //!< Active low
    DPS310_INT_HL_ACTIVE_HIGH = 1, //!< Active high
} dps310_int_hl_active_level;

/**
 * Mode of interupt when the FIFO is full.
 */
typedef enum {
    DPS310_INT_FIFO_DISABLE = 0, //!< Disable interupt when the FIFO is full
    DPS310_INT_FIFO_DNABLE  = 1, //!< Enable interupt when the FIFO is full
} dps310_int_fifo_mode_t;

/**
 * Mode of interupt when a temperature measurement is ready
 */
typedef enum {
    DSP310_INT_TMP_DISABLE  = 0, //!< Disable interupt when a temperature measurement is ready
    DSP310_INT_TMP_ENABLE   = 1, //!< Enable interupt when a temperature measurement is ready
} dps310_int_tmp_mode_t;

/**
 * Mode of interupt when a pressure measurement is ready
 */
typedef enum {
    DSP310_INT_PRS_DISABLE  = 0, //!< Disable interupt when a pressure measurement is ready
    DSP310_INT_PRS_ENABLE   = 1, //!< Enable interupt when a pressure measurement is ready
} dps310_int_pres_mode_t;

/**
 * Mode of temperature result bit-shift.
 */
typedef enum {
    DSP310_T_SHIFT_DISABLE  = 0, //!< No shift.
    DSP310_T_SHIFT_ENABLE   = 1, //!< Shift result right in data register.
                                 //   Must be set to '1' when the oversampling
                                 //   rate is >8 times.
} dps310_t_shift_mode_t;

/**
 * Mode of pressure result bit-shift.
 */
typedef enum {
    DSP310_P_SHIFT_DISABLE  = 0, //!< No shift.
    DSP310_P_SHIFT_ENABLE   = 1, //!< Shift result right in data register.
                                 //   Must be set to '1' when the oversampling
                                 //   rate is >8 times.
} dps310_p_shift_mode_t;

/**
 * Mode of FIFO.
 */
typedef enum {
    DSP310_FIFO_DISABLE = 0, //!< Disable FIFO.
    DSP310_FIFO_ENABLE  = 1, //!< Enable FIFO.
} dps310_fifo_en_mode_t;

/**
 * SPI mode.
 */
typedef enum {
    DSP310_SPI_MODE_4WIRE = 0, //!< SPI 4-wires
    DSP310_SPI_MODE_3WIRE = 1, //!< SPI 3-wires
} dsp310_spi_mode_t;

/**
 * Configuration parameters for DPS310.
 */
typedef struct {
    dps310_pm_rate_t pm_rate;
    dps310_pm_prc_t pm_prc;
    dps310_tmp_rate_t tmp_rate;
    dps310_tmp_prc_t tmp_prc;
    dps310_tmp_src_ext_t tmp_src;
    dps310_tmp_src_ext_t tmp_coef;
    dps310_int_fifo_mode_t int_fifo_mode;
    dps310_int_tmp_mode_t int_tmp_mode;
    dps310_int_pres_mode_t int_pres_mode;
    dps310_t_shift_mode_t t_shift_mode;
    dps310_p_shift_mode_t p_shift_mode;
    dps310_fifo_en_mode_t fifo_en_mode;
    dsp310_spi_mode_t spi_mode;

} dps310_config_t;

/**
 * A macro to set default dps310_config_t.
 */

#define DPS310_CONFIG_DEFAULT() { \
    .pm_rate = DPS310_PM_RATE_1, \
    .pm_prc = DPS310_PM_PRC_16, \
    .tmp_rate = DPS310_TMP_RATE_1, \
    .tmp_prc = DPS310_TMP_PRC_16, \
    .tmp_src = DPS310_TMP_SRC_EXTERNAL, \
    .tmp_coef = DPS310_TMP_SRC_EXTERNAL, \
    .int_fifo_mode = DPS310_INT_FIFO_DISABLE, \
    .int_tmp_mode = DSP310_INT_TMP_DISABLE, \
    .int_pres_mode = DSP310_INT_PRS_DISABLE, \
    .t_shift_mode = DSP310_T_SHIFT_ENABLE, \
    .p_shift_mode = DSP310_P_SHIFT_ENABLE, \
    .fifo_en_mode = DSP310_FIFO_ENABLE, \
    .spi_mode = DSP310_SPI_MODE_4WIRE, \
}

/**
 * Device descriptor.
 */
typedef struct {
    i2c_dev_t i2c_dev;          //!< I2C device descriptor
    uint8_t prod_id;            //!< Product ID
    uint8_t prod_rev;           //!< Product revision
} dps310_t;

/**
 * DPS310 registers
 */

/* 7 Register Map */
#define DPS310_REG_PSR_B2       0x00
#define DPS310_REG_PSR_B1       0x01
#define DPS310_REG_PSR_B0       0x02
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
#define DPS310_REG_COEF_SRCE    0x28

/* various masks */
#define DPS310_REG_ID_REV_MASK  0x0f
#define DPS310_REG_ID_PROD_MASK 0xf0
#define DPS310_REG_RESET_FIFO_FLUSH_MASK    (1 << 7)
#define DPS310_REG_RESET_SOFT_RST_MASK      0x0f
#define DPS310_REG_PRS_CFG_PM_RATE_SHIFT    (4)
#define DPS310_REG_PRS_CFG_PM_RATE_MASK     (0b111 << DPS310_REG_PRS_CFG_PM_RATE_SHIFT)
#define DPS310_REG_PRS_CFG_PM_PRC_MASK      (0b1111)

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

/* See 8.9 Soft Reset and FIFO flush (RESET) */
#define DPS310_FIFO_FLUSH_VALUE 0b1000
#define DPS310_SOFT_RST_VALUE   0b1001

/**
 * @brief Initialize device descriptor
 *
 * The device descriptor is dynamically allocated by the driver, and keeps its
 * internal states. The returned device descriptor is not public, and should
 * not be directly touched by user code. The device descriptor must be freed
 * by `dps310_free_desc()`.
 *
 * @param dev[out] Device descriptor. Must be NULL.
 * @param addr[in] DPS310 I2C address
 * @param port[in] I2C port number.
 *                 See available I2C port at:
 *                 https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#_CPPv410i2c_port_t
 * @param sda_gpio[in] GPIO pin for SDA
 * @param scl_gpio[in] GPIO pin for SCL
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
 * - read the DPS310_REG_ID, and identify the product ID. return ESP_FAIL if
 *   the product ID does not match expected product ID.
 * - reset the chip
 * - perform a quirk
 *
 * @param dev[in] Device descriptor
 * @param config[in] Configuration
 * @return `ESP_OK` on success
 */
esp_err_t dps310_init(dps310_t *dev, dps310_config_t *config);

/**
 * @brief Reset the device.
 *
 * Perform "soft reset" and ensure the chip is fully functional by delaying
 * `DPS310_STARTUP_DELAY_MS`.
 *
 * @param[in] dev Device descriptor
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `config` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_reset(dps310_t *dev);

/**
 * @brief Get pressure measurement rate.
 *
 * @param[in] dev pointer to the device descriptor
 * @param[out] value the value in the resister
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` and/or
 * `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_pm_rate(dps310_t *dev, uint8_t *value);

/**
 * @brief Set pressure measurement rate.
 *
 * @param[in] dev The pointer to the device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `config` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_pm_rate(dps310_t *dev, dps310_pm_rate_t value);

/**
 * @brief Get pressure oversampling rate.
 *
 * @param[in] dev pointer to the device descriptor
 * @param[out] value the value in the resister
 * @return `ESP_OK` on success. `ESP_ERR_INVALID_ARG` when `dev` and/or
 * `value` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_get_pm_prc(dps310_t *dev, uint8_t *value);

/**
 * @brief Set pressure oversampling rate.
 *
 * @param[in] dev The pointer to the device descriptor.
 * @param[in] value The value to set.
 * @return `ESP_OK` on success, `ESP_ERR_INVALID_ARG` when `dev` and/or `config` is NULL, or other errors when I2C communication fails.
 */
esp_err_t dps310_set_pm_prc(dps310_t *dev, dps310_pm_rate_t value);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __DPS310_H__

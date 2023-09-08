/*
 * Copyright (c) 2023 Jose Manuel Perez <jmpmscorp@hotmail.com>
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
 * @file sgm58031.h
 * @defgroup sgm58031 sgm58031
 * @{
 *
 * ESP-IDF driver for SGM58031 16-bit I2C ADC
 *
 * Copyright (c) 2023 Jose Manuel Perez <jmpmscorp@hotmail.com>
 */
#ifndef __SGM58031_H__
#define __SGM58031_H__

#include <stdbool.h>
#include <esp_err.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SGM58031_ADDR_GND (0x48) //!< I2C device address with ADDR pin connected to ground
#define SGM58031_ADDR_VCC (0x49) //!< I2C device address with ADDR pin connected to VCC
#define SGM58031_ADDR_SDA (0x4A) //!< I2C device address with ADDR pin connected to SDA
#define SGM58031_ADDR_SCL (0x4B) //!< I2C device address with ADDR pin connected to SCL

#define SGM58031_MAX_VALUE (0x7fff) //!< Maximum ADC value

/**
 * Gain amplifier
 */
typedef enum {
    SGM58031_GAIN_6V144 = 0, //!< +-6.144V
    SGM58031_GAIN_4V096,     //!< +-4.096V
    SGM58031_GAIN_2V048,     //!< +-2.048V (default)
    SGM58031_GAIN_1V024,     //!< +-1.024V
    SGM58031_GAIN_0V512,     //!< +-0.512V
    SGM58031_GAIN_0V256,     //!< +-0.256V
    SGM58031_GAIN_0V256_2,   //!< +-0.256V (same as ADS111X_GAIN_0V256)
    SGM58031_GAIN_0V256_3,   //!< +-0.256V (same as ADS111X_GAIN_0V256)
} sgm58031_gain_t;

/**
 * Gain amplifier values
 */
extern const float sgm58031_gain_values[];

/**
 * Input multiplexer configuration
 */
typedef enum {
    SGM58031_MUX_AIN0_AIN1 = 0, //!< positive = AIN0, negative = AIN1 (default)
    SGM58031_MUX_AIN0_AIN3,     //!< positive = AIN0, negative = AIN3
    SGM58031_MUX_AIN1_AIN3,     //!< positive = AIN1, negative = AIN3
    SGM58031_MUX_AIN2_AIN3,     //!< positive = AIN2, negative = AIN3
    SGM58031_MUX_AIN0_GND,      //!< positive = AIN0, negative = GND
    SGM58031_MUX_AIN1_GND,      //!< positive = AIN1, negative = GND
    SGM58031_MUX_AIN2_GND,      //!< positive = AIN2, negative = GND
    SGM58031_MUX_AIN3_GND,      //!< positive = AIN3, negative = GND
} sgm58031_mux_t;

/**
 * Data rate
 */
typedef enum {
    SGM58031_DATA_RATE_6_25 = 0x00, //!< 6.25 samples per second (DR_SEL = 0)
    SGM58031_DATA_RATE_12_5,        //!< 12.5 samples per second (DR_SEL = 0)
    SGM58031_DATA_RATE_25,          //!< 25 samples per second (DR_SEL = 0)
    SGM58031_DATA_RATE_50,          //!< 50 samples per second (DR_SEL = 0)
    SGM58031_DATA_RATE_100,         //!< 100 samples per second (DR_SEL = 0) (default)
    SGM58031_DATA_RATE_200,         //!< 200 samples per second (DR_SEL = 0)
    SGM58031_DATA_RATE_400,         //!< 400 samples per second (DR_SEL = 0)
    SGM58031_DATA_RATE_800,         //!< 800 samples per second (DR_SEL = 0)
    SGM58031_DATA_RATE_7_5,         //!< 7.5 samples per second (DR_SEL = 1)
    SGM58031_DATA_RATE_15,          //!< 15 samples per second (DR_SEL = 1)
    SGM58031_DATA_RATE_30,          //!< 30 samples per second (DR_SEL = 1)
    SGM58031_DATA_RATE_60,          //!< 60 samples per second (DR_SEL = 1)
    SGM58031_DATA_RATE_120,         //!< 120 samples per second (DR_SEL = 1)
    SGM58031_DATA_RATE_240,         //!< 240 samples per second (DR_SEL = 1)
    SGM58031_DATA_RATE_480,         //!< 480 samples per second (DR_SEL = 1)
    SGM58031_DATA_RATE_960          //!< 960 samples per second (DR_SEL = 1)
} sgm58031_data_rate_t;

/**
 * Device operating mode
 */
typedef enum {
    SGM58031_CONV_MODE_CONTINUOUS = 0, //!< Continuous conversion mode
    SGM58031_CONV_MODE_SINGLE_SHOT     //!< Power-down single-shot mode (default)
} sgm58031_conv_mode_t;

/**
 * Comparator mode
 */
typedef enum {
    SGM58031_COMP_MODE_NORMAL = 0, //!< Traditional comparator with hysteresis (default)
    SGM58031_COMP_MODE_WINDOW      //!< Window comparator
} sgm58031_comp_mode_t;

/**
 * Comparator polarity
 */
typedef enum {
    SGM58031_COMP_POLARITY_LOW = 0, //!< Active low (default)
    SGM58031_COMP_POLARITY_HIGH     //!< Active high
} sgm58031_comp_polarity_t;

/**
 * Comparator latch
 */
typedef enum {
    SGM58031_COMP_LATCH_DISABLED = 0, //!< Non-latching comparator (default)
    SGM58031_COMP_LATCH_ENABLED       //!< Latching comparator
} sgm58031_comp_latch_t;

/**
 * Comparator queue
 */
typedef enum {
    SGM58031_COMP_QUEUE_1 = 0,   //!< Assert ALERT/RDY pin after one conversion
    SGM58031_COMP_QUEUE_2,       //!< Assert ALERT/RDY pin after two conversions
    SGM58031_COMP_QUEUE_4,       //!< Assert ALERT/RDY pin after four conversions
    SGM58031_COMP_QUEUE_DISABLED //!< Disable comparator (default)
} sgm58031_comp_queue_t;

/**
 * @brief Initialize device descriptor
 *
 * @param[in] dev Device descriptor
 * @param[in] addr Device address
 * @param[in] port I2C port number
 * @param[in] sda_gpio GPIO pin for SDA
 * @param[in] scl_gpio GPIO pin for SCL
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param[in] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgm58031_free_desc(i2c_dev_t *dev);

/**
 * @brief Get device operational status
 *
 * @param[in] dev Device descriptor
 * @param[out] busy True when device performing conversion
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise */
esp_err_t sgm58031_is_busy(i2c_dev_t *dev, bool *busy);

/**
 * @brief Begin a single conversion
 *
 * Only in single-shot mode.
 *
 * @param[in] dev Device descriptor
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_start_conversion(i2c_dev_t *dev);

/**
 * @brief Read last conversion result
 *
 * @param[in] dev Device descriptor
 * @param[out] value Last conversion result
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_value(i2c_dev_t *dev, int16_t *value);

/**
 * @brief Read the programmable gain amplifier configuration
 *
 * Use ::sgm58031_gain_values[] for real voltage.
 *
 * @param[in] dev Device descriptor
 * @param[out] gain Gain value
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or gain are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_gain(i2c_dev_t *dev, sgm58031_gain_t *gain);

/**
 * @brief Configure the programmable gain amplifier
 *
 * @param[in] dev Device descriptor
 * @param[in] gain Gain value
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_gain(i2c_dev_t *dev, sgm58031_gain_t gain);

/**
 * @brief Read the input multiplexer configuration
 *
 * @param[in] dev Device descriptor
 * @param[out] mux Input multiplexer configuration
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or mux are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_input_mux(i2c_dev_t *dev, sgm58031_mux_t *mux);

/**
 * @brief Configure the input multiplexer configuration
 *
 * @param[in] dev Device descriptor
 * @param[in] mux Input multiplexer configuration
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_input_mux(i2c_dev_t *dev, sgm58031_mux_t mux);

/**
 * @brief Read the device operating conversion mode
 *
 * @param[in] dev Device descriptor
 * @param[out] mode Device operating mode
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or mode are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_conv_mode(i2c_dev_t *dev, sgm58031_conv_mode_t *mode);

/**
 * @brief Set the device operating mode
 *
 * @param[in] dev Device descriptor
 * @param[in] mode Device operating mode
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_conv_mode(i2c_dev_t *dev, sgm58031_conv_mode_t mode);

/**
 * @brief Read the data rate
 *
 * @param[in] dev Device descriptor
 * @param[out] rate Data rate
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or rate are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_data_rate(i2c_dev_t *dev, sgm58031_data_rate_t *rate);

/**
 * @brief Configure the data rate
 *
 * @param[in] dev Device descriptor
 * @param[in] rate Data rate
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_data_rate(i2c_dev_t *dev, sgm58031_data_rate_t rate);

/**
 * @brief Get comparator mode
 *
 * @param[in] dev Device descriptor
 * @param[out] mode Comparator mode
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or mode are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_comp_mode(i2c_dev_t *dev, sgm58031_comp_mode_t *mode);

/**
 * @brief Set comparator mode
 *
 * @param dev Device descriptor
 * @param mode Comparator mode
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_comp_mode(i2c_dev_t *dev, sgm58031_comp_mode_t mode);

/**
 * @brief Get polarity of the comparator output pin ALERT/RDY
 *
 * @param[in] dev Device descriptor
 * @param[out] polarity Comparator output pin polarity
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or polarity are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_comp_polarity(i2c_dev_t *dev, sgm58031_comp_polarity_t *polarity);

/**
 * @brief Set polarity of the comparator output pin ALERT/RDY
 *
 * @param[in] dev Device descriptor
 * @param[in] polarity Comparator output pin polarity
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_comp_polarity(i2c_dev_t *dev, sgm58031_comp_polarity_t polarity);

/**
 * @brief Get comparator output latch mode
 *
 * @param[in] dev Device descriptor
 * @param[out] latch Comparator output latch mode
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or latch are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_comp_latch(i2c_dev_t *dev, sgm58031_comp_latch_t *latch);

/**
 * @brief Set comparator output latch mode
 *
 * @param[in] dev Device descriptor
 * @param[in] latch Comparator output latch mode
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_comp_latch(i2c_dev_t *dev, sgm58031_comp_latch_t latch);

/**
 * @brief Get comparator queue size
 *
 * Get number of the comparator conversions before pin ALERT/RDY
 * assertion.
 *
 * @param[in] dev Device descriptor
 * @param[out] queue Number of the comparator conversions
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or queue are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_comp_queue(i2c_dev_t *dev, sgm58031_comp_queue_t *queue);

/**
 * @brief Set comparator queue size
 *
 * Set number of the comparator conversions before pin ALERT/RDY
 * assertion or disable comparator.
 *
 * @param[in] dev Device descriptor
 * @param[in] queue Number of the comparator conversions
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_comp_queue(i2c_dev_t *dev, sgm58031_comp_queue_t queue);

/**
 * @brief Get the lower threshold value used by comparator
 *
 * @param[in] dev Device descriptor
 * @param[out] th Lower threshold value
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or th are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_comp_low_thresh(i2c_dev_t *dev, int16_t *th);

/**
 * @brief Set the lower threshold value used by comparator
 *
 * @param[in] dev Device descriptor
 * @param[in] th Lower threshold value
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_comp_low_thresh(i2c_dev_t *dev, int16_t th);

/**
 * @brief Get the upper threshold value used by comparator
 *
 * @param[in] dev Device descriptor
 * @param[out] th Upper threshold value
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or th are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_comp_high_thresh(i2c_dev_t *dev, int16_t *th);

/**
 * @brief Set the upper threshold value used by comparator
 *
 * @param[in] dev Device descriptor
 * @param[in] th Upper threshold value
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_comp_high_thresh(i2c_dev_t *dev, int16_t th);

/**
 * @brief Enable/disable ain3 as external reference
 *
 * @param[in] dev Device descriptor
 * @param[in] enable True to enable. False to disable
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_ain3_external_reference(i2c_dev_t *dev, bool enable);

/**
 * @brief Get ain3 as external reference enable/disable bit
 *
 * @param[in] dev Device descriptor
 * @param[out] enable True, enable. False, disable
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_ain3_external_reference(i2c_dev_t *dev, bool *enable);

/**
 * @brief Enable/disable source pair of 2uA to selected pair of AINx
 *
 * @param[in] dev Device descriptor
 * @param[in] enable True to enable. False to disable
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_source_pair(i2c_dev_t *dev, bool enable);

/**
 * @brief Get burnout enable/disable bit
 *
 * @param[in] dev Device descriptor
 * @param[out] enable True, enable. False, disable
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_source_pair(i2c_dev_t *dev, bool *enable);

/**
 * @brief Enable/disable I2C bus leakage blocking circuit.
 *
 * @param[in] dev Device descriptor
 * @param[in] enable True to enable. False to disable
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_bus_leakage_circuit(i2c_dev_t *dev, bool enable);

/**
 * @brief Get I2C bus leakage blocking circuit enable/disable bit
 *
 * @param[in] dev Device descriptor
 * @param[out] enable True, enable. False, disable
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_bus_leakage_circuit(i2c_dev_t *dev, bool *enable);

/**
 * @brief Get chip id values
 *
 * @param[in] dev   Device descriptor
 * @param[out] id   ID value
 * @param[out] version  Version value
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev, id or version are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_chip_id(i2c_dev_t *dev, uint8_t *id, uint8_t *version);

/**
 * @brief Set GN Trim1 value
 *
 * @param[in] dev       Device descriptor
 * @param[in] trim_value Value to be set
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev is NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_set_gn_trim1(i2c_dev_t *dev, uint16_t trim_value);

/**
 * @brief Set GN Trim1 value
 *
 * @param[in] dev       Device descriptor
 * @param[in] trim_value Value to be set
 * @return
 *  - `ESP_OK` on success
 *  - `ESP_INVALID_ARG` if dev or trim_value are NULL
 *  - `ESP_FAIL` otherwise
 */
esp_err_t sgm58031_get_gn_trim1(i2c_dev_t *dev, uint16_t *trim_value);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SGM58031_H__ */

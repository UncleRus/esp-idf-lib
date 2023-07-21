/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Julian Doerner <https://github.com/juliandoerner>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file tsl2591.h
 * @defgroup tsl2591 tsl2591
 * @{
 *
 * ESP-IDF driver for TSL2591 light-to-digital. 
 *
 * Copyright (c) 2020 Julian Doerner <https://github.com/juliandoerner>
 *
 * MIT Licensed as described in the file LICENSE
 */

#ifndef __TSL2591_H__
#define __TSL2591_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSL2591_I2C_ADDR   0x29 // TSL2591 has only one i2c address.

/**
 * Power status. The sensor measures only if ALS an Power is on.
 */
typedef enum
{
    TSL2591_POWER_OFF = 0x00,
    TSL2591_POWER_ON = 0x01 //!< Default
} tsl2591_power_status_t;

/**
 * ALS status. The sensor measures only if ALS and Power is on.
 */
typedef enum
{
    TSL2591_ALS_OFF = 0x00,
    TSL2591_ALS_ON = 0x02   //!< Default
} tsl2591_als_status_t;

/**
 * Interrupts. TSL2591 has two interrupt sources. 
 * Check the datasheet for details.
 */
typedef enum
{
    TSL2591_INTR_OFF = 0x00, //!< Default
    TSL2591_ALS_INTR_ON = 0x10,
    TSL2591_ALS_INTR_NP_ON = 0x80,
    TSL2591_ALS_INTR_BOTH_ON = 0x90
} tsl2591_interrupt_t;

/**
 * Interrupt sleep setting. 
 */
typedef enum
{
    TSL2591_SLEEP_AFTER_OFF = 0x00, //!< Default
    TSL2591_SLEEP_AFTER_ON = 0x40
} tsl2591_sleep_after_intr_t;

/**
 * Integration time.
 */
typedef enum
{
    TSL2591_INTEGRATION_100MS = 0, //!< Default 
    TSL2591_INTEGRATION_200MS,      
    TSL2591_INTEGRATION_300MS,      
    TSL2591_INTEGRATION_400MS,      
    TSL2591_INTEGRATION_500MS,      
    TSL2591_INTEGRATION_600MS       
} tsl2591_integration_time_t;

/**
 * Gain.
 */
typedef enum
{
    TSL2591_GAIN_LOW = 0x00, //!< Default
    TSL2591_GAIN_MEDIUM = 0x10,
    TSL2591_GAIN_HIGH = 0x20,
    TSL2591_GAIN_MAX = 0x30
} tsl2591_gain_t;

/**
 * Persistence filter.
 */
typedef enum
{
    TSL2591_EVERY_CYCLE = 0, //!< Default
    TSL2591_NO_PERSIST,
    TSL2591_2_CYCLES,
    TSL2591_3_CYCLES,
    TSL2591_5_CYCLES,
    TSL2591_10_CYCLES,
    TSL2591_15_CYCLES,
    TSL2591_20_CYCLES,
    TSL2591_25_CYCLES,
    TSL2591_30_CYCLES,
    TSL2591_35_CYCLES,
    TSL2591_40_CYCLES,
    TSL2591_45_CYCLES,
    TSL2591_50_CYCLES,
    TSL2591_55_CYCLES,
    TSL2591_60_CYCLES
} tsl2591_persistence_filter_t;

/**
 * Device descriptor.
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    struct
    {
        uint8_t enable_reg;
        uint8_t control_reg;
        uint8_t persistence_reg;
    } settings;
} tsl2591_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO pin
 * @param scl_gpio SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_init_desc(tsl2591_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_free_desc(tsl2591_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_init(tsl2591_t *dev);

/**
 * @brief Read channel data
 *
 * @param dev Device descriptor
 * @param[out] channel0 Channel 0 data
 * @param[out] channel1 Channel 1 data
 */
esp_err_t tsl2591_get_channel_data(tsl2591_t *dev, uint16_t *channel0, uint16_t *channel1);

/**
 * @brief Calculate light intensity from channels
 *
 * @param dev Device descriptor
 * @param channel0 Channel0 data
 * @param channel1 Channel1 data
 * @param[out] lux Light intensity
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_calculate_lux(tsl2591_t *dev, uint16_t channel0, uint16_t channel1, float *lux);

/**
 * @brief Get and calculate light intensity
 *
 * @param dev Device descriptor
 * @param[out] lux Light intensity
 * @return `ESP_OK`
 */
esp_err_t tsl2591_get_lux(tsl2591_t *dev, float *lux);

/**
 * @brief Set device power status
 *
 * @param dev Device descriptor
 * @param power_status Power status
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_set_power_status(tsl2591_t *dev, tsl2591_power_status_t power_status);

/**
 * @brief Get device power status
 *
 * @param dev Device descriptor
 * @param[out] power_status Power status
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_get_power_status(tsl2591_t *dev, tsl2591_power_status_t *power_status);

/**
 * @brief Set device ALS status
 *
 * @param dev Device descriptor
 * @param als_status Als status
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_set_als_status(tsl2591_t *dev, tsl2591_als_status_t als_status);

/**
 * @brief Get device ALS status
 *
 * @param dev Device descriptor
 * @param[out] als_status Als status
 * @return `ESP_OK`
 */
esp_err_t tsl2591_get_als_status(tsl2591_t *dev, tsl2591_als_status_t *als_status);

/**
 * @brief Set device interrupt mode
 *
 * @param dev Device descriptor
 * @param interrupt interrupt mode
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_set_interrupt(tsl2591_t *dev, tsl2591_interrupt_t interrupt);

/**
 * @brief Get device interrupt mode
 *
 * @param dev Device descriptor
 * @param[out] interrupt interrupt mode
 * @return `ESP_OK`
 */
esp_err_t tsl2591_get_interrupt(tsl2591_t *dev, tsl2591_interrupt_t *interrupt);

/**
 * @brief Set sleeping after interrupt
 *
 * @param dev Device descriptor
 * @param sleep_after_intr Sleeping after interrupt
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_set_sleep_after_intr(tsl2591_t *dev, tsl2591_sleep_after_intr_t sleep_after_intr);

/**
 * @brief Get sleeping after interrupt setting
 *
 * @param dev Device descriptor
 * @param[out] sleep_after_intr Sleeping after interrupt
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_get_sleep_after_intr(tsl2591_t *dev, tsl2591_sleep_after_intr_t *sleep_after_intr);

/**
 * @brief Set device integration time
 *
 * @param dev Device descriptor
 * @param integration_time Integration time
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_set_integration_time(tsl2591_t *dev, tsl2591_integration_time_t integration_time);

/**
 * @brief Get device integration time
 *
 * @param dev Device descriptor
 * @param[out] integration_time Integration time
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_get_integration_time(tsl2591_t *dev, tsl2591_integration_time_t *integration_time);

/**
 * @brief Set device gain
 *
 * @param dev Device descriptor
 * @param gain Gain
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_set_gain(tsl2591_t *dev, tsl2591_gain_t gain);

/**
 * @brief Get device gain
 *
 * @param dev Device descriptor
 * @param[out] gain Gain
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_get_gain(tsl2591_t *dev, tsl2591_gain_t *gain);

/**
 * @brief Set device persistence filter
 *
 * @param dev Device descriptor
 * @param filter Persistence filter 
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_set_persistence_filter(tsl2591_t *dev, tsl2591_persistence_filter_t filter);

/**
 * @brief Get device persistence filter
 *
 * @param dev Device descriptor
 * @param[out] filter Persistence filter
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_get_persistence_filter(tsl2591_t *dev, tsl2591_persistence_filter_t *filter);

/**
 * @brief Set ALS interrupt low threshold
 *
 * @param dev Device descriptor
 * @param low_threshold Low threshold
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_als_set_low_threshold(tsl2591_t *dev, uint16_t low_threshold);

/**
 * @brief Set ALS interrupt high threshold
 *
 * @param dev Device descriptor
 * @param high_threshold High threshold
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_als_set_high_threshold(tsl2591_t *dev, uint16_t high_threshold);

/**
 * @brief Set no persist ALS interrupt low threshold
 *
 * @param dev Device descriptor
 * @param low_threshold Low threshold
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_no_persist_set_low_threshold(tsl2591_t *dev, uint16_t low_threshold);

/**
 * @brief Set no persist ALS interrupt high threshold
 *
 * @param dev Device descriptor
 * @param high_threshold High threshold
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_no_persist_set_high_threshold(tsl2591_t *dev, uint16_t high_threshold);

/**
 * @brief Set interrupt
 *
 * At least on interrupt must be enabled.
 *
 * @param dev Device descriptor
 * @return `ESP_PK` on success
 */
esp_err_t tsl2591_set_test_intr(tsl2591_t *dev);

/**
 * @brief Clear ALS interrupt
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_clear_als_intr(tsl2591_t *dev);

/**
 * @brief Clear ALS no persist interrupt
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_clear_als_np_intr(tsl2591_t *dev);

/**
 * @brief Clear both interrupts
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_clear_both_intr(tsl2591_t *dev);

/**
 * @brief Get ALS no persist interrupt flag
 *
 * @param dev Device descriptor
 * @param[out] flag Interrupt flag
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_get_np_intr_flag(tsl2591_t *dev, bool *flag);

/**
 * @brief Get ALS interrupt flag
 *
 * @param dev Device descriptor
 * @param[out] flag Interrupt flag
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_get_als_intr_flag(tsl2591_t *dev, bool *flag);

/**
 * @brief Get ALS validity flag
 *
 * This flag is set when integration cycle is completed
 * after enabling ALS.
 *
 * @param dev Device descriptor
 * @param[out] flag Validity flag
 * @return `ESP_OK` on success
 */
esp_err_t tsl2591_get_als_valid_flag(tsl2591_t *dev, bool *flag);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __TSL2591_H__

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Joshua Kallus <joshk.kallus3@gmail.com>
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
 * @file ls7366r.h
 *
 * @defgroup ls7366r ls7366r
 * @{
 *
 * ESP-IDF driver for LS7366R Quadrature Encoder Counter
 *
 * Datasheet: https://lsicsi.com/datasheets/LS7366R.pdf
 *
 * Copyright (c) 2021 Joshua Kallus <joshk.kallus3@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __LS7366R_H__
#define __LS7366R_H__

#include <driver/spi_master.h>
#include <esp_err.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LS7366R_MAX_CLOCK_SPEED_HZ (2000000) // 2 MHz clock

/**
 * Counter type
 */
typedef enum
{
    LS7366R_NON_QUAD, // Up down counting with channel A being step and channel B being direction
    LS7366R_1x_QUAD, // 1X quadrature
    LS7366R_2x_QUAD, // 2X quadrature
    LS7366R_4X_QUAD, // 4X quadrature (otherwise known as full quadrature)
} ls7366r_count_type_t;

/**
 * Count mode
 */
typedef enum
{
    LS7366R_FREE_RUN, // Free running count mode
    LS7366R_SINGLE_COUNT, // Single cycle count mode (counter disabled with carry or borrow, re-enabled with reset or load)
    LS7366R_RANGE_LIMIT, // Up and down count-ranges are limited between DTR and zero
    LS7366R_N_MODULO, // Input count clock frequency is divided by a factor of (n+1), where n = DTR, in both up and down directions
} ls7366r_count_mode_t;

/**
 * Index mode
 */
typedef enum
{
    LS7366R_INDEX_DISABLED, // Disable index
    LS7366R_INDEX_LOAD_CNTR, // Configure index as the "load CNTR" input (transfers DTR to CNTR)
    LS7366R_INDEX_RESET_CNTR, // Configure index as the "reset CNTR" input (clears CNTR to 0)
    LS7366R_INDEX_LOAD_OTR // Configure index as the "load OTR" input (transfers CNTR to OTR)
} ls7366r_index_mode_t;

/**
 * Index sync or async
 */
typedef enum
{
    LS7366R_INDEX_SYNCHRONOUS, // Asynchronous index
    LS7366R_INDEX_ASYNCHRONOUS // Synchronous index (overridden in non-quadrature mode)
} ls7366r_index_sync_t;

/**
 * Counter bits
 */
typedef enum
{
    LS7366R_8_BIT, // 1 byte counter mode
    LS7366R_16_BIT, // 2 byte counter mode
    LS7366R_24_BIT, // 3 byte counter mode
    LS7366R_32_BIT // 4 byte counter mode
} ls7366r_counter_bits_t;

/**
 * Counter clock divider
 */
typedef enum
{
    LS7366R_FILTER_CLK_1, // Filter clock division factor 1
    LS7366R_FILTER_CLK_2 // Filter clock division factor 2
} ls7366r_filter_clock_divider_t;

/**
 * Counter enable
 */
typedef enum
{
    LS7366R_COUNTER_ENABLE, // Enable counting
    LS7366R_COUNTER_DISABLE // Disable counting
} ls7366r_counter_enable_t;

/**
 * Counter flag carry mode
 */
typedef enum
{
    LS7366R_FLAG_CARRY_ENABLE, // Enable flag on carry bit
    LS7366R_FLAG_CARRY_DISABLE // Disable flag on carry bit
} ls7366r_flag_carry_mode_t;

/**
 * Counter flag borrow mode
 */
typedef enum
{
    LS7366R_FLAG_BORROW_ENABLE, // Enable flag on borrow bit
    LS7366R_FLAG_BORROW_DISABLE // Disable flag on borrow bit
} ls7366r_flag_borrow_mode_t;

/**
 * Counter flag index mode
 */
typedef enum
{
    LS7366R_FLAG_INDEX_ENABLE, // Enable flag on index bit
    LS7366R_FLAG_INDEX_DISABLE // Disable flag on index bit
} ls7366r_flag_index_mode_t;

/**
 * Counter flag compare mode
 */
typedef enum
{
    LS7366R_FLAG_COMPARE_ENABLE, // Enable flag on compare bit
    LS7366R_FLAG_COMPARE_DISABLE // Disable flag on compare bit
} ls7366r_flag_compare_enable_t;

typedef struct
{
    ls7366r_flag_borrow_mode_t borrow;
    ls7366r_flag_carry_mode_t carry;
    ls7366r_flag_compare_enable_t compare;
    ls7366r_flag_index_mode_t index;
} ls7366r_flag_mode_t; // LS7366R flag mode struct

/**
 * Device descriptor
 */
typedef struct
{
    spi_device_interface_config_t spi_cfg;
    spi_device_handle_t spi_dev;
} ls7366r_t; // LS7366R device struct

/**
 * Device configuration
 */
typedef struct
{
    ls7366r_flag_mode_t flag_mode; // MDR1
    ls7366r_counter_enable_t counter_enable; // MDR1
    ls7366r_counter_bits_t counter_bits; // MDR1
    ls7366r_filter_clock_divider_t filter_clock_divider; // MDR0
    ls7366r_index_sync_t index_sync; // MDR0
    ls7366r_index_mode_t index_mode; // MDR0
    ls7366r_count_type_t count_type; // MDR0
} ls7366r_config_t; // LS7366R config struct

/**
 * @brief Initialize device descriptor
 *
 * @param dev            Device descriptor
 * @param host           SPI host
 * @param clock_speed_hz SPI clock speed, Hz
 * @param cs_pin         CS GPIO number
 * @return `ESP_OK` on success
 */
esp_err_t ls7366r_init_desc(ls7366r_t *dev, spi_host_device_t host, uint32_t clock_speed_hz, gpio_num_t cs_pin);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ls7366r_free_desc(ls7366r_t *dev);

/**
 * @brief Configure device
 *
 * @param dev    Device descriptor
 * @param config Configuration
 * @return `ESP_OK` on success
 */
esp_err_t ls7366r_set_config(ls7366r_t *dev, const ls7366r_config_t *config);

/**
 * @brief Get current count
 *
 * @param dev    Device descriptor
 * @param count  Count variable
 * @return `ESP_OK` on success
 */
esp_err_t ls7366r_get_count(ls7366r_t *dev, int32_t *count);

/**
 * @brief set value for compare
 *
 * @param dev    Device descriptor
 * @param cmp    Compare value
 * @return `ESP_OK` on success
 */
esp_err_t ls7366r_set_compare_val(ls7366r_t *dev, int32_t cmp);

/**
 * @brief clear counter value, set to 0
 *
 * @param dev    Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ls7366r_clear_counter(ls7366r_t *dev);

/**
 * @brief enable or disable counter
 *
 * @param dev    Device descriptor
 * @param enable Counter enabled or disabled
 * @return `ESP_OK` on success
 */
esp_err_t ls7366r_counter_enable(ls7366r_t *dev, bool enable);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LS7366R_H__ */

/*
 * Copyright (c) 2022 Jaime Albuquerque <jaime.albq@gmail.com>
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
 * @file shift_reg.h
 * @defgroup shift_reg shift_reg
 * @{
 * 
 * ESP-IDF driver for generic shift register using 3 pins:
 * - data
 * - clock
 * - latch
 *
 */

#if !defined(__SHIFT_REG__H__)
#define __SHIFT_REG__H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <esp_err.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <ets_sys.h>

/**
 * @brief Direction of the data
 * 
 * @todo Add Input mode
 * 
 */
typedef enum {
    SHIFT_DIR_OUTPUT = 0,
} shift_reg_dir_t;

/**
 * @brief Register orientation mode
 * 
 */
typedef enum {
    SHIFT_BIT_MODE_LSB = 0,
    SHIFT_BIT_MODE_MSB,
} shift_reg_bit_mode_t;

/**
 * @brief This structure stores the mode of the shift register
 * 
 */
typedef struct {
    shift_reg_dir_t         dir         : 2;    // Direction mode of the shift register
    shift_reg_bit_mode_t    bit_mode    : 1;    // Bit mode
} shift_reg_mode_t;

/**
 * @brief This structure stores all used pins to interface with the shift register
 * 
 */
typedef struct {
    gpio_num_t              clk;                // Clock pin
    gpio_num_t              data;               // Data/Signal pin
    gpio_num_t              latch;              // Latch pin
} shift_reg_pin_t;

/**
 * @brief This structure stores all needed configuration of the shift register component
 * 
 */
typedef struct {                                    // Configuration of shift register
    uint8_t                     num_reg;            // Number of shift registers
    uint8_t                     *reg_value;         // Last value of all registers
    shift_reg_mode_t            mode;               // Direction adn bit mode
    shift_reg_pin_t             pin;                // Set of pins
} shift_reg_config_t;

/**
 * @brief       Initialize the microcontroller to do the output
 *
 * @param dev  shift register configuration
 * @return      `ESP_OK` on success
 */
esp_err_t shift_reg_init(shift_reg_config_t *dev);

/**
 * @brief       Free used memory
 * 
 * @param dev  shift register configuration
 * @return      `ESP_OK` on success
 */
esp_err_t shift_reg_deinit(shift_reg_config_t *dev);

/**
 * @brief       Send the whole data
 * 
 * @param dev  shift register configuration
 * @param data  data vector
 * @param len   length of the data
 * @return      `ESP_OK` on success
 */
esp_err_t shift_reg_send(shift_reg_config_t *dev, uint8_t *data, uint8_t len);

/**
 * @brief       Send 1 byte of data
 * 
 * @param dev  shift register configuration
 * @param data  1 byte data
 * @return      `ESP_OK` on success
 */
esp_err_t shift_reg_send8bits(shift_reg_config_t *dev, uint8_t data);

/**
 * @brief       Latch the data inside of the shift register
 * 
 * @param dev  shift register configuration
 * @return      `ESP_OK` on success
 */
esp_err_t shift_reg_latch(shift_reg_config_t *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __SHIFT_REG__H__

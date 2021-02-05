/*
 * Copyright (c) 2019 Tomoyuki Sakurai <y@trombik.org>
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
 * @file lm75.h
 * @defgroup lm75 lm75
 * @{
 *
 * ESP-IDF driver for LM75, a digital temperature sensor and thermal watchdog.
 *
 * The driver depends on i2cdev library in `esp-idf-lib`.
 *
 * The driver was written using LM75B.
 *
 * Short usage instruction:
 *
 * 1. Include lm75.h
 * 2. Initialize I2C descriptor by i2cdev_init()
 * 3. Initialize LM75 descriptor by lm75_init_desc()
 * 4. Initialize LM75 by lm75_init()
 * 5. Read temperature by lm75_read_temperature()
 *
 */
#ifndef __LM75_H__
#define __LM75_H__

#include <stdint.h>
#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LM75_I2C_ADDRESS_DEFAULT (0x48) //!< Default I2C address (A0 == A1 == A2 == 0)
#define LM75_I2C_ADDRESS_MAX (0x4f)     //!< I2C address (A0 == A1 == A2 == 1)

/**
 * Operation mode of LM75
 */
typedef enum {
    LM75_MODE_NORMAL = 0,  //!< Normal operation mode
    LM75_MODE_SHUTDOWN = 1 //!< Shutdown mode
} lm75_mode_t;

/**
 * Overtemperature Shutdown Polarity
 */
typedef enum {
    LM75_OSP_LOW  = 0, //!< Overtemperature Shutdown Polarity is active low
    LM75_OSP_HIGH = 1  //!< OSP is active high
} lm75_os_polarity_t;

/**
 * Overtemperature Shutdown output mode
 */
typedef enum {
    LM75_OS_MODE_COMP = 0, //!< OS output mode is comparator
    LM75_OS_MODE_INT  = 1  //!< OS output mode is interrupt
} lm75_os_mode_t;

/**
 *  OS fault queue, the number of faults that must occur consecutively to
 *  activate the OS output
 */
typedef enum {
    LM75_FAULT_QUEUE1 = 0b00, //!< 1
    LM75_FAULT_QUEUE2 = 0b01, //!< 2
    LM75_FAULT_QUEUE4 = 0b10, //!< 4
    LM75_FAULT_QUEUE6 = 0b11  //!< 6
} lm75_fault_queue_t;

/**
 * Device configuration
 */
typedef struct {
    lm75_mode_t mode;                     //!< Operation mode of the device
    lm75_os_polarity_t os_pol;            //!< OS Polarity
    lm75_os_mode_t os_mode;               //!< OS mode
    lm75_fault_queue_t os_fault_queue;    //!< OS fault queue
} lm75_config_t;

/**
 * @brief Initialize LM75 device descriptor
 *
 * i2cdev_init() must be called before this function.
 *
 * @param[out] dev pointer to LM75 device descriptor
 * @param[in] addr I2C address of LM75
 * @param[in] port I2C port
 * @param[in] sda_gpio GPIO number of SDA
 * @param[in] scl_gpio GPIO number of SCL
 * @return `ESP_OK` on success
 */
esp_err_t lm75_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Initialize LM75
 *
 * lm75_init_desc() must be called before this function.
 *
 * @param[in] dev pointer to LM75 device descriptor
 * @param[in] config configuration
 */
esp_err_t lm75_init(i2c_dev_t *dev, const lm75_config_t config);

/**
 * @brief free LM75 device descriptor
 * @param dev Pointer to device descriptor
 */
esp_err_t lm75_free_desc(i2c_dev_t *dev);

/**
 * @brief Get the value of OS Polarity in the configuration register
 * @param[in] dev pointer to LM75 device descriptor
 * @param[out] v value of OS Polarity
 * @return `ESP_OK` on success
 */
esp_err_t lm75_get_os_polarity(i2c_dev_t *dev, uint8_t *v);

/**
 * @brief Get the value of OS threshold in the configuration register
 * @param[in] dev pointer to LM75 device descriptor
 * @param[out] value value of OS threshold
 * @return `ESP_OK` on success
 */
esp_err_t lm75_get_os_threshold(i2c_dev_t *dev, float *value);

/**
 * @brief Read the temperature
 * @param[in] dev pointer to LM75 device descriptor
 * @param[out] value temperature
 * @return `ESP_OK` on success
 */
esp_err_t lm75_read_temperature(i2c_dev_t *dev, float *value);

/**
 * @brief Set OS mode
 * @param[in] dev pointer to LM75 device descriptor
 * @param[in] v OS mode
 * @return `ESP_OK` on success
 */
esp_err_t lm75_set_os_mode(i2c_dev_t *dev, const lm75_os_mode_t v);

/**
 * @brief Set the value of OS Polarity in the configuration register
 *
 * @param[in] dev pointer to LM75 device descriptor
 * @param[in] v value of OS Polarity
 * @return `ESP_OK` on success
 */
esp_err_t lm75_set_os_polarity(i2c_dev_t *dev, const lm75_os_polarity_t v);

/**
 * @brief Set the value of OS threshold in the configuration register
 * @param[in] dev pointer to LM75 device descriptor
 * @param[in] value value of OS threshold
 * @return `ESP_OK` on success
 */
esp_err_t lm75_set_os_threshold(i2c_dev_t *dev, const float value);

/**
 * @brief Shutdown LM75
 * @param[in] dev pointer to LM75 device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t lm75_shutdown(i2c_dev_t *dev);

/**
 * @brief Wake LM75 up
 * @param[in] dev pointer to LM75 device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t lm75_wakeup(i2c_dev_t *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif

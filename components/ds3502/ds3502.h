/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file ds3502.h
 * @defgroup ds3502 ds3502
 * @{
 *
 * ESP-IDF driver for nonvolatile digital potentiometer DS3502
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DS3502_H__
#define __DS3502_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DS3502_ADDR_0 0x28
#define DS3502_ADDR_1 0x29
#define DS3502_ADDR_2 0x2a
#define DS3502_ADDR_3 0x2b

/**
 * Maximal wiper position value
 */
#define DS3502_MAX 0x7f

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr Device address, `DS3502_ADDR_...`
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t ds3502_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ds3502_free_desc(i2c_dev_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ds3502_init(i2c_dev_t *dev);

/**
 * @brief Get wiper position
 *
 * @param dev Device descriptor
 * @param[out] pos Position, `0..DS3502_MAX_WIPER`
 * @return `ESP_OK` on success
 */
esp_err_t ds3502_get(i2c_dev_t *dev, uint8_t *pos);

/**
 * @brief Set wiper position
 *
 * @param dev Device descriptor
 * @param pos Wiper position, `0..DS3502_MAX_WIPER`
 * @param save Save position to nonvolatile memory
 * @return `ESP_OK` on success
 */
esp_err_t ds3502_set(i2c_dev_t *dev, uint8_t pos, bool save);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __DS3502_H__ */

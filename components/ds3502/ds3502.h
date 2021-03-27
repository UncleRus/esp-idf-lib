/**
 * @file ds3502.h
 * @defgroup ds3502 ds3502
 * @{
 *
 * ESP-IDF driver for nonvolatile digital potentiometer DS3502
 *
 * Copyright (C) 2021 Ruslan V. Uss <unclerus@gmail.com>
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

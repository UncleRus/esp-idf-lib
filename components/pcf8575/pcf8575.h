/**
 * @file pcf8575.h
 * @defgroup pcf8575 pcf8575
 * @{
 *
 * ESP-IDF driver for PCF8575 remote 16-bit I/O expander for I2C-bus
 *
 * Copyright (C) 2019 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __PCF8575_H__
#define __PCF8575_H__

#include <stddef.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCF8575_I2C_ADDR_BASE 0x20

/**
 * @brief Initialize device descriptior
 *
 * SCL frequency is 400kHz
 *
 * @param dev Pointer to I2C device descriptor
 * @param port I2C port number
 * @param addr I2C address (`0b0100<A2><A1><A0>` for PCF8575)
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t pcf8575_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf8575_free_desc(i2c_dev_t *dev);

/**
 * @brief Read GPIO port value
 * @param dev Pointer to I2C device descriptor
 * @param val 8-bit GPIO port value
 * @return `ESP_OK` on success
 */
esp_err_t pcf8575_port_read(i2c_dev_t *dev, uint16_t *val);

/**
 * @brief Write value to GPIO port
 * @param dev Pointer to I2C device descriptor
 * @param value GPIO port value
 * @return ESP_OK on success
 */
esp_err_t pcf8575_port_write(i2c_dev_t *dev, uint16_t value);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PCF8575_H__ */

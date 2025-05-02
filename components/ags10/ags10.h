/**
 * @file ags10.h
 * @brief AGS10 sensor driver header file
 * @author xyzroe
 * ESP-IDF driver for AGS10 sensor
 *
 * Copyright (c) 2024 xyzroe <i@xyzroe.cc>
 * Licensed as described in the file LICENSE
 */

#ifndef _AGS10_H
#define _AGS10_H

#include <stdint.h>
#include <stdbool.h>
#include <driver/i2c.h>
#include <i2cdev.h>

#define AGS10_I2CADDR_DEFAULT 0x1A  ///< AGS10 default I2C address
#define AGS10_TVOCSTAT_REG    0x00  ///< Status and TVOC reading
#define AGS10_VERSION_REG     0x11  ///< Firmware version
#define AGS10_GASRES_REG      0x20  ///< Raw gas resistance
#define AGS10_SETADDR_REG     0x21  ///< Change I2C address
#define AGS10_CRC8_INIT       0xFF  ///< CRC8 init value
#define AGS10_CRC8_POLYNOMIAL 0x31  ///< CRC8 polynomial
#define I2C_FREQ_HZ           20000 ///< Fixed I2C frequency for AGS10

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the AGS10 sensor descriptor
 *
 * @param dev Pointer to the I2C device descriptor
 * @param port I2C port number
 * @param addr I2C address of the sensor
 * @param sda_gpio GPIO number for SDA
 * @param scl_gpio GPIO number for SCL
 * @return ESP_OK on success
 */
esp_err_t ags10_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free the AGS10 sensor descriptor
 *
 * @param dev Pointer to the I2C device descriptor
 * @return ESP_OK on success
 */
esp_err_t ags10_free_desc(i2c_dev_t *dev);

/**
 * @brief Read TVOC value from the sensor
 *
 * @param dev Pointer to the I2C device descriptor
 * @param[out] tvoc Pointer to store the TVOC value
 * @return ESP_OK on success
 */
esp_err_t ags10_read_tvoc(i2c_dev_t *dev, uint32_t *tvoc);

/**
 * @brief Read firmware version from the sensor
 *
 * @param dev Pointer to the I2C device descriptor
 * @param[out] version Pointer to store the firmware version
 * @return ESP_OK on success
 */
esp_err_t ags10_read_version(i2c_dev_t *dev, uint8_t *version);

/**
 * @brief Read resistance value from the sensor
 *
 * @param dev Pointer to the I2C device descriptor
 * @param[out] resistance Pointer to store the resistance value
 * @return ESP_OK on success
 */
esp_err_t ags10_read_resistance(i2c_dev_t *dev, uint32_t *resistance);

/**
 * @brief Set new I2C address for the sensor
 *
 * @param dev Pointer to the I2C device descriptor
 * @param new_address New I2C address
 * @return ESP_OK on success
 */
esp_err_t ags10_set_i2c_address(i2c_dev_t *dev, uint8_t new_address);

/**
 * @brief Set zero-point calibration with factory defaults
 *
 * @param dev Pointer to the I2C device descriptor
 * @return ESP_OK on success
 */
esp_err_t ags10_set_zero_point_with_factory_defaults(i2c_dev_t *dev);

/**
 * @brief Set zero-point calibration with current resistance
 *
 * @param dev Pointer to the I2C device descriptor
 * @return ESP_OK on success
 */
esp_err_t ags10_set_zero_point_with_current_resistance(i2c_dev_t *dev);

/**
 * @brief Set zero-point calibration with a specific value
 *
 * @param dev Pointer to the I2C device descriptor
 * @param value Calibration value
 * @return ESP_OK on success
 */
esp_err_t ags10_set_zero_point_with(i2c_dev_t *dev, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* _AGS10_H */
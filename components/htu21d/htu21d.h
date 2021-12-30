/*
 * Copyright (c) 2022 saasaa <mail@saasaa.xyz>
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
 * @file htu21d.h
 * @brief HTU21D(F) driver
 * @author saasaa
 * @defgroup htu21d htu21d
 * @{
 *
 * ESP-IDF driver for HTU21D(F) temperature and humidity sensor
 *
 * Datasheet: https://www.mouser.at/datasheet/2/418/5/NG_DS_HPC199_6_A1-1128627.pdf
 *
 * Copyright (c) 2022 saasaa <mail@saasaa.xyz>
 *
 * ISC Licensed as described in the file LICENSE
 *
 */

#ifndef __HTU21D_H__
#define __HTU21D_H__

#include <esp_err.h>
#include <i2cdev.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HTU21D_I2C_ADDRESS 0x40

typedef enum
{
    HTU21D_RES_T_14B_RH_12B = 0x00, //!< default
    HTU21D_RES_T_12B_RH_8B = 0x01,
    HTU21D_RES_T_13B_RH_10B = 0x80,
    HTU21D_RES_T_11B_RH_11B = 0x81,
} htu21d_resolution_t;

typedef enum
{
    HTU21D_BATTERY_OK = 0,
    HTU21D_BATTERY_LOW
} htu21d_battery_status_t;

typedef struct
{
    i2c_dev_t i2c_dev;
    htu21d_resolution_t res;
} htu21d_t;

/**
 * @brief Initialize the HTU21D(F) Device descriptor
 *
 * @param[out] dev Device descriptor
 * @param[in] addr I2C address
 * @param[in] port I2C port number
 * @param[in] sda_gpio GPIO pin number for SDA
 * @param[in] scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_init_desc(htu21d_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param[in] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_free_desc(htu21d_t *dev);

/**
 * @brief Initialize the HTU21D(F) sensor
 *
 * @param[in] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_init(htu21d_t *dev);

/**
 * @brief Soft reset the device and wait for 15ms
 *
 * @param[in] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_soft_reset(htu21d_t *dev);

/**
 * @brief Get temperature and relative humidity
 *
 * Currently using hold master mode and blocking the I2C bus for up to 66ms while measuring.
 *
 * @param[in] dev Device descriptor
 * @param[out] temperature Temperature, degrees Celsius
 * @param[out] humidity    relative humidity, percent
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_read_data(htu21d_t *dev, float *t, float *rh);

/**
 * @brief Get temperature. Holds the I2C bus for up to 50ms while measuring.
 *
 * @param[in] dev Device descriptor
 * @param[out] temperature Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_read_temperature_hold(htu21d_t *dev, float *t);

/**
 * @brief Get relative humidity. Holds the I2C bus for up to 16ms while measuring.
 *
 * @param[in] dev Device descriptor
 * @param[out] temperature Humidity, %RH
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_read_humidity_hold(htu21d_t *dev, float *rh);

/**
 * @brief Get temperature. Does not hold the I2C bus.
 *
 * @param[in] dev Device descriptor
 * @param[out] temperature Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_read_temperature_no_hold(htu21d_t *dev, float *t);

/**
 * @brief Get humidity. Does not hold the I2C bus.
 *
 * @param[in] dev Device descriptor
 * @param[out] humidity    Relative humidity, percents
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_read_humidity_no_hold(htu21d_t *dev, float *rh);

/**
 * @brief Get resolution setting
 *
 * @param[in,out] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_get_resolution(htu21d_t *dev);

/**
 * @brief Set resolution
 *
 * @param[in,out] dev Device descriptor
 * @param[in] res desired resolution
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_set_resolution(htu21d_t *dev, htu21d_resolution_t res);

/**
 * @brief Read battery status, battery_status = 0 means VDD > 2.25V, battery_status = 1 means VDD < 2.25V
 *
 * @param[in] dev Device descriptor
 * @param[out] battery_status Battery status
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_read_battery_status(htu21d_t *dev, htu21d_battery_status_t *battery_status);

/**
 * @brief Read current heater state
 *
 * @param[in] dev Device descriptor
 * @param[out] heater desired heater state
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_get_heater(htu21d_t *dev, bool *enable);

/**
 * @brief Switch heater on/off
 *
 * @param dev Device descriptor
 * @param enable true to switch heater on
 * @return `ESP_OK` on success
 */
esp_err_t htu21d_set_heater(htu21d_t *dev, bool enable);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __HTU21D_H__

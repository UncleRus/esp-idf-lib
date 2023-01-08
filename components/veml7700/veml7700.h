/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2022 Marc Luehr <marcluehr@gmail.com>
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
 * @file veml7700.h
 * @defgroup veml7700 veml7700
 * @{
 *
 * ESP-IDF driver for VEML7700 brightness sensors for I2C-bus
 *
 * Copyright (c) 2022 Marc Luehr <marcluehr@gmail.com>
 *
 * ISC Licensed as described in the file LICENSE
 */
#ifndef __VEML7700_H__
#define __VEML7700_H__

#include <stddef.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VEML7700_I2C_ADDR (0x10)

#define VEML7700_INTEGRATION_TIME_25MS  (0b1100)
#define VEML7700_INTEGRATION_TIME_50MS  (0b1000)
#define VEML7700_INTEGRATION_TIME_100MS (0b0000)
#define VEML7700_INTEGRATION_TIME_200MS (0b0001)
#define VEML7700_INTEGRATION_TIME_400MS (0b0010)
#define VEML7700_INTEGRATION_TIME_800MS (0b0011)

#define VEML7700_GAIN_1     (0b00)
#define VEML7700_GAIN_2     (0b01)
#define VEML7700_GAIN_DIV_8 (0b10)
#define VEML7700_GAIN_DIV_4 (0b11)

#define VEML7700_POWER_SAVING_MODE_500MS  (0b00)
#define VEML7700_POWER_SAVING_MODE_1000MS (0b01)
#define VEML7700_POWER_SAVING_MODE_2000MS (0b10)
#define VEML7700_POWER_SAVING_MODE_4000MS (0b11)

#define VEML7700_PERSISTENCE_PROTECTION_1 (0b00)
#define VEML7700_PERSISTENCE_PROTECTION_2 (0b01)
#define VEML7700_PERSISTENCE_PROTECTION_4 (0b10)
#define VEML7700_PERSISTENCE_PROTECTION_8 (0b11)

/**
 * VEML configuration descriptor
 */
typedef struct
{
    uint16_t gain : 2;                //!< control the sensitivity
    uint16_t integration_time : 4;    //!< time to measure
    uint16_t persistence_protect : 2; //!< sample count before the interrupt triggers
    uint16_t interrupt_enable : 1;    //!< enable threshold interrupt
    uint16_t shutdown : 1;            //!< set to 1 to shutdown the device, set to 0 to wakeup
    uint16_t threshold_high;          //!< high threshold for the interrupt
    uint16_t threshold_low;           //!< low threshold for the interrupt
    uint16_t power_saving_mode : 2;   //!< power saving mode
    uint16_t power_saving_enable : 1; //!< enable the pover saving mode
} veml7700_config_t;

/**
 * @brief Initialize device descriptor
 *
 * Default SCL frequency is 100kHz. The I2C address is fix.
 *
 * @param dev Pointer to I2C device descriptor
 * @param port I2C port number
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t veml7700_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t veml7700_free_desc(i2c_dev_t *dev);

/**
 * @brief Probe if the device exist on the bus
 *
 * @param dev Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t veml7700_probe(i2c_dev_t *dev);

/**
 * @brief Write the config to the device
 *
 * @param dev Pointer to I2C device descriptor
 * @param config Pointer to the config descriptor
 * @return `ESP_OK` on success
 */
esp_err_t veml7700_set_config(i2c_dev_t *dev, veml7700_config_t *config);

/**
 * @brief Read the config to the device
 *
 * @param dev Pointer to I2C device descriptor
 * @param config Pointer to the config descriptor
 * @return `ESP_OK` on success
 */
esp_err_t veml7700_get_config(i2c_dev_t *dev, veml7700_config_t *config);

/**
 * @brief Read ambient light sensor value from the device
 *
 * @param dev Pointer to I2C device descriptor
 * @param config Pointer to the config descriptor
 * @param value_lux Pointer as return value in lux
 * @return `ESP_OK` on success
 */
esp_err_t veml7700_get_ambient_light(i2c_dev_t *dev, veml7700_config_t *config, uint32_t *value_lux);

/**
 * @brief Read white channel value from the device
 *
 * @param dev Pointer to I2C device descriptor
 * @param config Pointer to the config descriptor
 * @param value_lux Pointer as return value in lux
 * @return `ESP_OK` on success
 */
esp_err_t veml7700_get_white_channel(i2c_dev_t *dev, veml7700_config_t *config, uint32_t *value_lux);

/**
 * @brief Read the interrupt status from the device
 *
 * @param dev Pointer to I2C device descriptor
 * @param low_threshold Pointer to return the low threshold passed indicator
 * @param high_threshold Pointer to return the high threshold passed indicator
 * @return `ESP_OK` on success
 */
esp_err_t veml7700_get_interrupt_status(i2c_dev_t *dev, bool *low_threshold, bool *high_threshold);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __VEML7700_H__ */

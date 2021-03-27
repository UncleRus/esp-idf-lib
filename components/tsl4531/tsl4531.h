/**
 * @file tsl4531.h
 * @defgroup tsl4531 tsl4531
 * @{
 *
 * ESP-IDF driver for digital ambient light sensor TSL4531
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2017 Brian Schwind <https://github.com/bschwind>\n
 * Copyright (C) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __TSL4531_H__
#define __TSL4531_H__

#include <stdint.h>
#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSL4531_I2C_ADDR 0x29

/**
 * Integration time
 */
typedef enum
{
    TSL4531_INTEGRATION_400MS = 0x00, //!< Default
    TSL4531_INTEGRATION_200MS = 0x01,
    TSL4531_INTEGRATION_100MS = 0x02,
} tsl4531_integration_time_t;

/**
 * Part IDs
 */
typedef enum
{
    TSL4531_PART_TSL45317 = 0x08,
    TSL4531_PART_TSL45313 = 0x09,
    TSL4531_PART_TSL45315 = 0x0A,
    TSL4531_PART_TSL45311 = 0x0B
} tsl4531_part_id_t;

/**
 * Device descriptor
 */
typedef struct {
    i2c_dev_t i2c_dev;
    tsl4531_integration_time_t integration_time;
    bool skip_power_save;
    tsl4531_part_id_t part_id;
} tsl4531_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO pin
 * @param scl_gpio SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t tsl4531_init_desc(tsl4531_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl4531_free_desc(tsl4531_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl4531_init(tsl4531_t *dev);

/**
 * @brief Configure device
 *
 * @param dev Device descriptor
 * @param integration_time Integration time
 * @param skip_power_save PowerSave Mode. When true, the power save states are
 *        skipped following a light integration cycle for shorter sampling rates
 * @return `ESP_OK` on success
 */
esp_err_t tsl4531_config(tsl4531_t *dev, tsl4531_integration_time_t integration_time, bool skip_power_save);

/**
 * @brief Read conversion results in lux
 *
 * @param dev Device descriptor
 * @param[out] lux Conversion result in lux
 * @return `ESP_OK` on success
 */
esp_err_t tsl4531_read_lux(tsl4531_t *dev, uint16_t *lux);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __TSL4531_H__

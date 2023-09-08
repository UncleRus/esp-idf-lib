/**
 * @file sgp40.h
 * @defgroup sgp40 sgp40
 * @{
 *
 * ESP-IDF driver for SGP40 Indoor Air Quality Sensor for VOC Measurements
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SGP40_H__
#define __SGP40_H__

#include <stdbool.h>
#include <time.h>
#include <i2cdev.h>
#include <esp_err.h>
#include "sensirion_voc_algorithm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SGP40_ADDR 0x59 //!< I2C address

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    uint16_t serial[3];
    uint16_t featureset;
    VocAlgorithmParams voc;
} sgp40_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t sgp40_init_desc(sgp40_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp40_free_desc(sgp40_t *dev);

/**
 * @brief Read device information, initialize the VOC algorithm
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp40_init(sgp40_t *dev);

/**
 * @brief Reset device, than put it to idle mode
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp40_soft_reset(sgp40_t *dev);

/**
 * @brief Perform a self-test
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp40_self_test(sgp40_t *dev);

/**
 * @brief Turn hotplate off, stop measurement and put device to idle mode
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp40_heater_off(sgp40_t *dev);

/**
 * @brief Perform a measurement
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                    you want uncompensated measurement
 * @param[out] raw Raw value, proportional to the logarithm
 *                 of the resistance of the sensing element
 * @return `ESP_OK` on success
 */
esp_err_t sgp40_measure_raw(sgp40_t *dev, float humidity, float temperature, uint16_t *raw);

/**
 * @brief Perform a measurement and update VOC index
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                    you want uncompensated measurement
 * @param[out] voc_index Calculated VOC index
 * @return
 */
esp_err_t sgp40_measure_voc(sgp40_t *dev, float humidity, float temperature, int32_t *voc_index);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SGP40_H__ */

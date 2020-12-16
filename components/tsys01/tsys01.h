/**
 * @file tsys01.h
 * @defgroup tsys01 tsys01
 * @{
 *
 * ESP-IDF driver for Digital Temperature Sensors TSYS01
 *
 * Copyright (C) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __TSYS01_H__
#define __TSYS01_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSYS01_I2C_ADDR1 0x76
#define TSYS01_I2C_ADDR2 0x77

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev; //!< I2C device descriptor
    uint16_t cal[8];    //!< Calibration values
} tsys01_t;

/**
 * @brief Initialize device descriptor.
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_init_desc(tsys01_t *dev, uint8_t addr, i2c_port_t port,
                           gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_free_desc(tsys01_t *dev);

/**
 * @brief Initialize device.
 *
 * Reads sensor configuration.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_init(tsys01_t *dev);

/**
 * @brief Reset sensor.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_reset(tsys01_t *dev);

/**
 * @brief Read temperature from sensor.
 *
 * This function starts temperature conversion,
 * waits 10 ms and reads result.
 *
 * @param dev Device descriptor
 * @param[out] t Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t tsys01_get_temperature(tsys01_t *dev, float *t);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __TSYS01_H__ */

/**
 * @file bmp180.h
 * @defgroup bmp180 bmp180
 * @{
 *
 * ESP-IDF driver for BMP180 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2015 Frank Bargstedt\n
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BMP180_H__
#define __BMP180_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#define BMP180_DEVICE_ADDRESS 0x77 //!< I2C address

#ifdef __cplusplus
extern "C" {
#endif

/**
 * BMP180 device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;

    int16_t  AC1;
    int16_t  AC2;
    int16_t  AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;

    int16_t  B1;
    int16_t  B2;

    int16_t  MB;
    int16_t  MC;
    int16_t  MD;
} bmp180_dev_t;

/**
 * Hardware accuracy mode.
 * See Table 3 of the datasheet
 */
typedef enum
{
    BMP180_MODE_ULTRA_LOW_POWER = 0,  //!< 1 sample, 4.5 ms
    BMP180_MODE_STANDARD,             //!< 2 samples, 7.5 ms
    BMP180_MODE_HIGH_RESOLUTION,      //!< 4 samples, 13.5 ms
    BMP180_MODE_ULTRA_HIGH_RESOLUTION //!< 8 samples, 25.5 ms
} bmp180_mode_t;


/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t bmp180_init_desc(bmp180_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Pointer to BMP180 device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t bmp180_free_desc(bmp180_dev_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Pointer to BMP180 device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t bmp180_init(bmp180_dev_t *dev);

/**
 * @brief Measure temperature and pressure
 *
 * @param dev Pointer to BMP180 device descriptor
 * @param[out] temperature Temperature in degrees Celsius
 * @param[out] pressure Pressure in Pa
 * @param oss Measurement mode
 * @return `ESP_OK` on success
 */
esp_err_t bmp180_measure(bmp180_dev_t *dev, float *temperature, uint32_t *pressure, bmp180_mode_t oss);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __BMP180_H__ */

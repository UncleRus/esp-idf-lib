/**
 * @file bmp180.h
 *
 * ESP-IDF driver for BMP180 digital pressure sensor
 *
 *  Created on: 23.08.2015
 *      Author: fbargste
 *  Ported from esp-open-rtos by UncleRus 2018
 */
#ifndef DRIVER_BMP180_H_
#define DRIVER_BMP180_H_

#include <stdbool.h>
#include <i2c_utils.h>

#define BMP180_DEVICE_ADDRESS 0x77

#define BMP180_TEMPERATURE (1<<0)
#define BMP180_PRESSURE    (1<<1)

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
 * Setup I2C bus.
 * I2C master, 1000000 Hz
 * @param scl_pin GPIO for SCL pin
 * @param sda_pin GPIO for SDA pin
 * @return ESP_OK on success
 */
esp_err_t bmp180_i2c_init(i2c_dev_t *dev, gpio_num_t scl_pin, gpio_num_t sda_pin);

/**
 * Init bmp180 driver
 * @param dev Pointer to BMP180 device descriptor
 * @return ESP_OK on success
 */
esp_err_t bmp180_init(bmp180_dev_t *dev);

/**
 * Check BMP180 availability
 * @param port I2C port number
 * @return true if bmp180 is available
 */
bool bmp180_is_available(i2c_port_t port);

/**
 * Measure temperature and pressure
 * @param dev Pointer to BMP180 device descriptor
 * @param temperature Temperature in degrees Celsius
 * @param pressure Pressure in MPa
 * @param oss Measurement mode
 * @return ESP_OK on success
 */
esp_err_t bmp180_measure(bmp180_dev_t *dev, float *temperature, uint32_t *pressure, bmp180_mode_t oss);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BMP180_H_ */

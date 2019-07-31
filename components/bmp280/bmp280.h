/**
 * @file bmp280.h
 * @defgroup bmp280 bmp280
 * @{
 *
 * ESP-IDF driver for BMP280/BME280 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016 sheinz <https://github.com/sheinz>\n
 * Copyright (C) 2018 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BMP280_H__
#define __BMP280_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMP280_I2C_ADDRESS_0  0x76 //!< I2C address when SDO pin is low
#define BMP280_I2C_ADDRESS_1  0x77 //!< I2C address when SDO pin is high

#define BMP280_CHIP_ID  0x58 //!< BMP280 has chip-id 0x58
#define BME280_CHIP_ID  0x60 //!< BME280 has chip-id 0x60

/**
 * Mode of BMP280 module operation.
 */
typedef enum {
    BMP280_MODE_SLEEP = 0,  //!< Sleep mode
    BMP280_MODE_FORCED = 1, //!< Measurement is initiated by user
    BMP280_MODE_NORMAL = 3  //!< Continues measurement
} BMP280_Mode;

typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

/**
 * Pressure oversampling settings
 */
typedef enum {
    BMP280_SKIPPED = 0,          //!< no measurement
    BMP280_ULTRA_LOW_POWER = 1,  //!< oversampling x1
    BMP280_LOW_POWER = 2,        //!< oversampling x2
    BMP280_STANDARD = 3,         //!< oversampling x4
    BMP280_HIGH_RES = 4,         //!< oversampling x8
    BMP280_ULTRA_HIGH_RES = 5    //!< oversampling x16
} BMP280_Oversampling;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    BMP280_STANDBY_05 = 0,      //!< stand by time 0.5ms
    BMP280_STANDBY_62 = 1,      //!< stand by time 62.5ms
    BMP280_STANDBY_125 = 2,     //!< stand by time 125ms
    BMP280_STANDBY_250 = 3,     //!< stand by time 250ms
    BMP280_STANDBY_500 = 4,     //!< stand by time 500ms
    BMP280_STANDBY_1000 = 5,    //!< stand by time 1s
    BMP280_STANDBY_2000 = 6,    //!< stand by time 2s BMP280, 10ms BME280
    BMP280_STANDBY_4000 = 7,    //!< stand by time 4s BMP280, 20ms BME280
} BMP280_StandbyTime;

/**
 * Configuration parameters for BMP280 module.
 * Use function bmp280_init_default_params to use default configuration.
 */
typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_Oversampling oversampling_humidity;
    BMP280_StandbyTime standby;
} bmp280_params_t;

/**
 * Device descriptor
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Humidity compensation for BME280 */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    i2c_dev_t i2c_dev;  //!< I2C device descriptor
    uint8_t   id;       //!< Chip ID
} bmp280_t;

/**
 * @brief Initialize device descriptior
 * @param[out] dev Pointer to device descriptor
 * @param[in] addr BMP280 address
 * @param[in] port I2C port number
 * @param[in] sda_gpio GPIO pin for SDA
 * @param[in] scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t bmp280_init_desc(bmp280_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Pointer to device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t bmp280_free_desc(bmp280_t *dev);

/**
 * Initialize default parameters.
 * Default configuration:
 *      mode: NORAML
 *      filter: OFF
 *      oversampling: x4
 *      standby time: 250ms
 */
esp_err_t bmp280_init_default_params(bmp280_params_t *params);

/**
 * Initialize BMP280 module, probes for the device, soft resets the device,
 * reads the calibration constants, and configures the device using the supplied
 * parameters. Returns `ESP_OK` on success.
 *
 * This may be called again to soft reset the device and initialize it again.
 */
esp_err_t bmp280_init(bmp280_t *dev, bmp280_params_t *params);

/**
 * Start measurement in forced mode.
 * The module remains in forced mode after this call.
 * Do not call this method in normal mode.
 */
esp_err_t bmp280_force_measurement(bmp280_t *dev);

/**
 * Check if BMP280 is busy with measuring temperature/pressure.
 * Return true if BMP280 is busy.
 */
esp_err_t bmp280_is_measuring(bmp280_t *dev, bool *busy);

/**
 * Read compensated temperature and pressure data:
 *
 *  Temperature in degrees Celsius times 100.
 *
 *  Pressure in Pascals in fixed point 24 bit integer 8 bit fraction format.
 *
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity as a fixed point 22 bit interger and 10 bit fraction format.
 */
esp_err_t bmp280_read_fixed(bmp280_t *dev, int32_t *temperature,
                            uint32_t *pressure, uint32_t *humidity);

/**
 * Read compensated temperature and pressure data:
 *  Temperature in degrees Celsius.
 *  Pressure in Pascals.
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity.
 */
esp_err_t bmp280_read_float(bmp280_t *dev, float *temperature,
                            float *pressure, float *humidity);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BMP280_H__

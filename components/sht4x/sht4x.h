/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file sht4x.h
 * @defgroup sht4x sht4x
 * @{
 *
 * ESP-IDF driver for Sensirion SHT40/SHT41 digital temperature and humidity sensor
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SHT4X_H__
#define __SHT4X_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHT4X_I2C_ADDRESS 0x44

#define SHT4X_RAW_DATA_SIZE 6

typedef uint8_t sht4x_raw_data_t[SHT4X_RAW_DATA_SIZE];

/**
 * Possible heater modes
 */
typedef enum
{
    SHT4X_HEATER_OFF = 0,      /**< Heater is off, default */
    SHT4X_HEATER_HIGH_LONG,    /**< High power (~200mW), 1 second pulse */
    SHT4X_HEATER_HIGH_SHORT,   /**< High power (~200mW), 0.1 second pulse */
    SHT4X_HEATER_MEDIUM_LONG,  /**< Medium power (~110mW), 1 second pulse */
    SHT4X_HEATER_MEDIUM_SHORT, /**< Medium power (~110mW), 0.1 second pulse */
    SHT4X_HEATER_LOW_LONG,     /**< Low power (~20mW), 1 second pulse */
    SHT4X_HEATER_LOW_SHORT,    /**< Low power (~20mW), 0.1 second pulse */
} sht4x_heater_t;

/**
 * Possible repeatability modes
 */
typedef enum
{
    SHT4X_HIGH = 0,
    SHT4X_MEDIUM,
    SHT4X_LOW
} sht4x_repeat_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;            //!< I2C device descriptor

    uint32_t serial;              //!< device serial number

    sht4x_heater_t heater;        //!< used measurement mode
    sht4x_repeat_t repeatability; //!< used repeatability

    bool meas_started;            //!< indicates whether measurement started
    uint64_t meas_start_time;     //!< measurement start time in us
} sht4x_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param port      I2C port
 * @param sda_gpio  SDA GPIO
 * @param scl_gpio  SCL GPIO
 * @return          `ESP_OK` on success
 */
esp_err_t sht4x_init_desc(sht4x_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht4x_free_desc(sht4x_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht4x_init(sht4x_t *dev);

/**
 * @brief Soft-reset sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht4x_reset(sht4x_t *dev);

/**
 * @brief High level measurement function
 *
 * For convenience this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. Starts a measurement
 * 2. Waits using `vTaskDelay()` until measurement results are available
 * 3. Returns the results in kind of floating point sensor values
 *
 * This function is the easiest way to use the sensor. It is most suitable
 * for users that don't want to have the control on sensor details.
 *
 * @note The function delays the calling task up to 1.1 s to wait for
 *       the measurement results. This might lead to problems when the function
 *       is called from a software timer callback function.
 *
 * @param dev         Device descriptor
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sht4x_measure(sht4x_t *dev, float *temperature, float *humidity);

/**
 * @brief Start the measurement.
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht4x_start_measurement(sht4x_t *dev);

/**
 * @brief Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability and heater setting.
 * Once a measurement is started with function ::sht4x_start_measurement()
 * the user task can use this duration in RTOS ticks directly to wait
 * with function `vTaskDelay()` until the measurement results can be fetched.
 *
 * @param dev       Device descriptor
 * @return          Measurement duration given in RTOS ticks
 */
size_t sht4x_get_measurement_duration(sht4x_t *dev);

/**
 * @brief Read measurement results from sensor as raw data
 *
 * The function read measurement results from the sensor, checks the CRC
 * checksum and stores them in the byte array as following.
 *
 *      data[0] = Temperature MSB
 *      data[1] = Temperature LSB
 *      data[2] = Temperature CRC
 *      data[3] = Humidity MSB
 *      data[4] = Humidity LSB
 *      data[5] = Humidity CRC
 *
 * In case that there are no new data that can be read, the function fails.
 *
 * @param dev      Device descriptor
 * @param[out] raw Byte array in which raw data are stored
 * @return         `ESP_OK` on success
 */
esp_err_t sht4x_get_raw_data(sht4x_t *dev, sht4x_raw_data_t raw);

/**
 * @brief Computes sensor values from raw data
 *
 * @param raw_data         Byte array that contains raw data
 * @param[out] temperature Temperature in degree Celsius
 * @param[out] humidity    Humidity in percent
 * @return                 `ESP_OK` on success
 */
esp_err_t sht4x_compute_values(sht4x_raw_data_t raw_data, float *temperature, float *humidity);

/**
 * @brief Get measurement results in form of sensor values
 *
 * The function combines function ::sht4x_get_raw_data() and function
 * ::sht4x_compute_values() to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 *
 * @param dev              Device descriptor
 * @param[out] temperature Temperature in degree Celsius
 * @param[out] humidity    Humidity in percent
 * @return                 `ESP_OK` on success
 */
esp_err_t sht4x_get_results(sht4x_t *dev, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SHT4X_H__ */

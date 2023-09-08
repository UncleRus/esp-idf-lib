/*
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file sht3x.h
 * @defgroup sht3x sht3x
 * @{
 *
 * ESP-IDF driver for Sensirion SHT3x digital temperature and humidity sensor
 *
 * Forked from <https://github.com/gschorcht/sht3x-esp-idf>
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SHT3X_H__
#define __SHT3X_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHT3X_I2C_ADDR_GND 0x44
#define SHT3X_I2C_ADDR_VDD 0x45

#define SHT3X_RAW_DATA_SIZE 6

typedef uint8_t sht3x_raw_data_t[SHT3X_RAW_DATA_SIZE];

/**
 * Possible measurement modes
 */
typedef enum
{
    SHT3X_SINGLE_SHOT = 0,  //!< one single measurement
    SHT3X_PERIODIC_05MPS,   //!< periodic with 0.5 measurements per second (mps)
    SHT3X_PERIODIC_1MPS,    //!< periodic with   1 measurements per second (mps)
    SHT3X_PERIODIC_2MPS,    //!< periodic with   2 measurements per second (mps)
    SHT3X_PERIODIC_4MPS,    //!< periodic with   4 measurements per second (mps)
    SHT3X_PERIODIC_10MPS    //!< periodic with  10 measurements per second (mps)
} sht3x_mode_t;

/**
 * Possible repeatability modes
 */
typedef enum
{
    SHT3X_HIGH = 0,
    SHT3X_MEDIUM,
    SHT3X_LOW
} sht3x_repeat_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;            //!< I2C device descriptor

    sht3x_mode_t mode;            //!< used measurement mode
    sht3x_repeat_t repeatability; //!< used repeatability

    bool meas_started;            //!< indicates whether measurement started
    uint64_t meas_start_time;     //!< measurement start time in us
    bool meas_first;              //!< first measurement in periodic mode
} sht3x_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param port      I2C port
 * @param addr      Device address
 * @param sda_gpio  SDA GPIO
 * @param scl_gpio  SCL GPIO
 * @return          `ESP_OK` on success
 */
esp_err_t sht3x_init_desc(sht3x_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht3x_free_desc(sht3x_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht3x_init(sht3x_t *dev);

/**
 * @brief Enable/disable heater
 *
 * @param dev       Device descriptor
 * @param enable    True to enable, false to disable
 * @return          `ESP_OK` on success
 */
esp_err_t sht3x_set_heater(sht3x_t *dev, bool enable);

/**
 * @brief High level measurement function
 *
 * For convenience this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. Starts a measurement in single shot mode with high reliability
 * 2. Waits using `vTaskDelay()` until measurement results are available
 * 3. Returns the results in kind of floating point sensor values
 *
 * This function is the easiest way to use the sensor. It is most suitable
 * for users that don't want to have the control on sensor details.
 *
 * @note The function delays the calling task up to 30 ms to wait for
 *       the measurement results. This might lead to problems when function
 *       is called from a software timer callback function.
 *
 * @param dev         Device descriptor
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sht3x_measure(sht3x_t *dev, float *temperature, float *humidity);

/**
 * @brief Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability. Once a measurement is
 * started with function ::sht3x_start_measurement() the user task can use this
 * duration in RTOS ticks directly to wait with function `vTaskDelay()` until
 * the measurement results can be fetched.
 *
 * @note The duration only depends on repeatability level. Therefore,
 *       it can be considered as constant for a repeatability.
 *
 * @param repeat    Repeatability, see type ::sht3x_repeat_t
 * @return          Measurement duration given in RTOS ticks
 */
uint8_t sht3x_get_measurement_duration(sht3x_repeat_t repeat);

/**
 * @brief Start the measurement in single shot or periodic mode
 *
 * The function starts the measurement either in *single shot mode*
 * (exactly one measurement) or *periodic mode* (periodic measurements)
 * with given repeatability.
 *
 * In the *single shot mode*, this function has to be called for each
 * measurement. The measurement duration has to be waited every time
 * before the results can be fetched.
 *
 * In the *periodic mode*, this function has to be called only once. Also
 * the measurement duration has to be waited only once until the first
 * results are available. After this first measurement, the sensor then
 * automatically performs all subsequent measurements. The rate of periodic
 * measurements can be 10, 4, 2, 1 or 0.5 measurements per second (mps).
 *
 * @note Due to inaccuracies in timing of the sensor, the user task
 *       should fetch the results at a lower rate. The rate of the periodic
 *       measurements is defined by the parameter \p mode.
 *
 * @param dev       Device descriptor
 * @param mode      Measurement mode, see type ::sht3x_mode_t
 * @param repeat    Repeatability, see type ::sht3x_repeat_t
 * @return          `ESP_OK` on success
 */
esp_err_t sht3x_start_measurement(sht3x_t *dev, sht3x_mode_t mode, sht3x_repeat_t repeat);

/**
 * @brief Stop the periodic mode measurements
 *
 * The function stops the measurements  in *periodic mode*
 * (periodic measurements) and the sensor returns in *single shot mode*
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht3x_stop_periodic_measurement(sht3x_t *dev);

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
 *      data[2] = Humidity CRC
 *
 * In case that there are no new data that can be read, the function fails.
 *
 * @param dev       Device descriptor
 * @param raw_data  Byte array in which raw data are stored
 * @return          `ESP_OK` on success
 */
esp_err_t sht3x_get_raw_data(sht3x_t *dev, sht3x_raw_data_t raw_data);

/**
 * @brief Computes sensor values from raw data
 *
 * @param raw_data    Byte array that contains raw data
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sht3x_compute_values(sht3x_raw_data_t raw_data, float *temperature, float *humidity);

/**
 * @brief Get measurement results in form of sensor values
 *
 * The function combines function ::sht3x_get_raw_data() and function
 * ::sht3x_compute_values() to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 *
 * @param dev         Device descriptor
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sht3x_get_results(sht3x_t *dev, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SHT3X_H__ */

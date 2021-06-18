/*
 * Copyright (c) 2021, Sensirion AG
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2021 Nate Usher <n.usher87@gmail.com>
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
 * @file scd30.h
 * @defgroup scd30 scd30
 * @{
 *
 * ESP-IDF driver for Sensirion SCD30 CO2 sensor.
 *
 * Adapted from https://github.com/UncleRus/esp-idf-lib/tree/master/components/scd4x
 * 
 * Copyright (c) 2021, Sensirion AG
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2021 Nate Usher <n.usher87@gmail.com>
 * 
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SCD30_H__
#define __SCD30_H__

#include <i2cdev.h>
#include <esp_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCD30_I2C_ADDR 0x61

/**
 * @brief Initialize device descriptor.
 *
 * @param dev      Device descriptor
 * @param port     I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd30_free_desc(i2c_dev_t *dev);

/**
 * @brief Trigger continuous  measurement.
 *
 * Signal update interval default is 2 seconds.
 *
 * @param dev           Device descriptor
 * @param p_comp        Optional ambient pressure compensation in mBar, 0 to deactivate 
 * @return              `ESP_OK` on success
 */
esp_err_t scd30_trigger_continuous_measurement(i2c_dev_t *dev, uint16_t p_comp);

/**
 * @brief Stop continuous measurement.
 *
 * Stops the continuous measurement of the SCD30.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd30_stop_continuous_measurement(i2c_dev_t *dev);

/**
 * @brief Get measurement interval
 *
 * Gets the interval used bythe SCD30 sensor to measure in continuous measurement mode.
 * Saved in non-volatile memory.
 *
 * @param dev                   Device descriptor
 * @param interval_seconds      Measurement interval in seconds 
 * 
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_get_measurement_interval(i2c_dev_t *dev, uint16_t *interval_seconds);

/**
 * @brief Set measurement interval
 *
 * Sets the interval used bythe SCD30 sensor to measure in continuous measurement mode.
 * Saved in non-volatile memory.
 *
 * @param dev                   Device descriptor
 * @param interval_seconds      Measurement interval in seconds 
 * 
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_set_measurement_interval(i2c_dev_t *dev, uint16_t interval_seconds);

/**
 * @brief Check whether new measurement data is available for read-out.
 *
 * @param dev        Device descriptor
 * @param data_ready true if data is ready, false otherwise
 * @return           `ESP_OK` on success
 */
esp_err_t scd30_get_data_ready_status(i2c_dev_t *dev, bool *data_ready);

/**
 * @brief Read sensor output and convert.
 *
 * When new measurement data is available it can be read out with the following command.
 * Make sure that the measurement is completed by calling scd30_get_data_ready_status()
 *
 * @param dev         Device descriptor
 * @param co2         CO₂ concentration in ppm
 * @param temperature Temperature in degrees Celsius (°C)
 * @param humidity    Relative humidity in percent RH
 * @return            `ESP_OK` on success
 */
esp_err_t scd30_read_measurement(i2c_dev_t *dev, float *co2, float *temperature, float *humidity);

/**
 * @brief Get automatic self calibration (ASC) state.
 *
 * By default, the ASC is disabled.
 *
 * @param dev     Device descriptor.
 * @param enabled true if ASC is enabled, false otherwise
 * @return        `ESP_OK` on success
 */
esp_err_t scd30_get_automatic_self_calibration(i2c_dev_t *dev, bool *enabled);

/**
 * @brief Enable or disable automatic self calibration (ASC).
 *
 * By default, the ASC is disabled.
 *
 * @param dev     Device descriptor.
 * @param enabled true to enable ASC, false to disable ASC
 * @return        `ESP_OK` on success
 */
esp_err_t scd30_set_automatic_self_calibration(i2c_dev_t *dev, bool enabled);

/**
 * @brief Get Forced Recalibration Value
 * 
 * See scd_30_set_forced_recalibration_value.
 * 
 * The most recently used reference value is retained in volatile memory and 
 * can be read out with the command sequence given below. After repowering 
 * the sensor, the command will return the standard reference value of 400 ppm.
 * 
 * @param dev                      Device descriptor.
 * @param correction_value         FRC correction value in CO₂ ppm 
 * 
 * @return                         `ESP_OK` on success
 */
esp_err_t scd30_get_forced_recalibration_value(i2c_dev_t *dev,
        uint16_t *correction_value);

/**
 * @brief Set Forced Recalibration Value.
 *
 * Forced recalibration (FRC) is used to compensate for sensor drifts when a reference
 * value of the CO2 concentration in close proximity to the SCD30 is available.
 * For best results,the sensor has to be run in a stable environment in continuous
 * mode at a measurement rateof 2s for at least two minutes before applying the FRC
 * command and sending the reference value. Setting a reference CO2 concentration 
 * by the method described here will always supersede corrections from the ASC
 * 
 * Imposes a permanent update to the calibration curve which persists after repowering
 * the sensor.
 * @param dev                      Device descriptor.
 * @param target_co2_concentration Target CO₂ concentration in ppm. (400 <= val <= 2000)
 *                                 
 * @return                         `ESP_OK` on success
 */
esp_err_t scd30_set_forced_recalibration_value(i2c_dev_t *dev,
        uint16_t target_co2_concentration);

/**
 * @brief Get temperature offset in ticks.
 *
 * Get the current temperature offset value saved in non-volatile memory.  
 *
 * @note Only available in idle mode.
 *
 * @param dev      Device descriptor
 * @param t_offset Temperature offset.
 *                 Convert value to °C by: value * 100; 
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_get_temperature_offset_ticks(i2c_dev_t *dev, uint16_t *t_offset);

/**
 * @brief Get temperature offset in °C.
 *
 * See ::scd30_get_temperature_offset_ticks() for more details.
 *
 * @param dev      Device descriptor
 * @param t_offset Temperature offset in degrees Celsius (°C)
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_get_temperature_offset(i2c_dev_t *dev, float *t_offset);

/**
 * @brief Set temperature offset in ticks.
 *
 * Set the temperature offset value to be saved in non-volatile memory.
 * The last set value will be used for temperature offset compensation after repowering.
 *
 * @param dev      Device descriptor
 * @param t_offset Temperature offset.
 *                 Convert °C to value by: T / 100;
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_set_temperature_offset_ticks(i2c_dev_t *dev, uint16_t t_offset);

/**
 * @brief Set temperature offset in °C.
 *
 * See ::scd30_set_temperature_offset_ticks() for more details.
 *
 * @param dev      Device descriptor
 * @param t_offset Temperature offset in degrees Celsius (°C)
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_set_temperature_offset(i2c_dev_t *dev, float t_offset);

/**
 * @brief Get configured sensor altitude.
 *
 * Get configured sensor altitude in meters above sea level. Per default, the
 * sensor altitude is set to 0 meter above sea-level.
 *
 * @note Only available in idle mode.
 *
 * @param dev      Device descriptor.
 * @param altitude Sensor altitude in meters.
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_get_sensor_altitude(i2c_dev_t *dev, uint16_t *altitude);

/**
 * @brief Set sensor altitude in meters above sea level.
 *
 * Note that setting a sensor altitude to the sensor overrides any pressure
 * compensation based on a previously set ambient pressure.
 *
 * @param dev      Device descriptor.
 * @param altitude Sensor altitude in meters.
 * @return         `ESP_OK` on success
 */
esp_err_t scd30_set_sensor_altitude(i2c_dev_t *dev, uint16_t altitude);

/**
 * @brief Get firmware version.
 *
 * Following command can be used to read out the firmware version of SCD30 module
 * The MSB is the major firmware version, the LSB is the minor firmware version 
 * @param dev                   Device descriptor
 * @param firmware_version      Firmware version 
 * 
 * @return        `ESP_OK` on success
 */
esp_err_t scd30_read_firmware_version(i2c_dev_t *dev, uint16_t *firmware_version);

/**
 * @brief Reset the sensor
 *
 * Soft reset mechanism that forces the sensor into the same state as after powering up
 * After soft reset the sensor will reload all calibrated data. 
 * 
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd30_soft_reset(i2c_dev_t *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SCD3O_H__ */

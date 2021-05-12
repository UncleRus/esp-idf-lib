/*
 * Copyright (c) 2021, Sensirion AG
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
 * @file scd4x.h
 * @defgroup scd4x scd4x
 * @{
 *
 * ESP-IDF driver for SCD4x CO2 sensor.
 *
 * Ported from https://github.com/Sensirion/raspberry-pi-i2c-scd4x
 *
 * Copyright (c) 2021, Sensirion AG
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SCD4X_H__
#define __SCD4X_H__

#include <i2cdev.h>
#include <esp_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCD4X_I2C_ADDR 0x62

/**
 * @brief Initialize device descriptor.
 *
 * @param dev      Device descriptor
 * @param port     I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return         `ESP_OK` on success
 */
esp_err_t scd4x_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_free_desc(i2c_dev_t *dev);

/**
 * @brief Start periodic measurement.
 *
 * Signal update interval is 5 seconds.
 *
 * @note This command is only available in idle mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_start_periodic_measurement(i2c_dev_t *dev);

/**
 * @brief Read sensor output.
 *
 * The measurement data can only be read out once per signal update interval
 * as the buffer is emptied upon read-out. If no data is available in the
 * buffer, the sensor returns a NACK. To avoid a NACK response the
 * ::scd4x_get_data_ready_status() can be called to check data status.
 *
 * @note This command is only available in measurement mode. The firmware
 * updates the measurement values depending on the measurement mode.
 *
 * @param dev         Device descriptor
 * @param co2         CO₂ concentration in ppm
 * @param temperature Convert value to °C by: -45 °C + 175 °C * value/2^16
 * @param humidity    Convert value to %RH by: 100%RH * value/2^16
 * @return            `ESP_OK` on success
 */
esp_err_t scd4x_read_measurement_ticks(i2c_dev_t *dev, uint16_t *co2, uint16_t *temperature, uint16_t *humidity);

/**
 * @brief Read sensor output and convert.
 *
 * See ::scd4x_read_measurement_ticks() for more details.
 *
 * @note This command is only available in measurement mode. The firmware
 * updates the measurement values depending on the measurement mode.
 *
 * @param dev         Device descriptor
 * @param co2         CO₂ concentration in ppm
 * @param temperature Temperature in degrees Celsius (°C)
 * @param humidity    Relative humidity in percent RH
 * @return            `ESP_OK` on success
 */
esp_err_t scd4x_read_measurement(i2c_dev_t *dev, uint16_t *co2, float *temperature, float *humidity);

/**
 * @brief Stop periodic measurement.
 *
 * Stop periodic measurement and return to idle mode for sensor configuration
 * or to safe energy.
 *
 * @note This command is only available in measurement mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_stop_periodic_measurement(i2c_dev_t *dev);

/**
 * @brief Get temperature offset in ticks.
 *
 * The temperature offset represents the difference between the measured
 * temperature by the SCD4x and the actual ambient temperature. Per default,
 * the temperature offset is set to 4°C.
 *
 * @note Only available in idle mode.
 *
 * @param dev      Device descriptor
 * @param t_offset Temperature offset.
 *                 Convert value to °C by: 175 * value / 2^16
 * @return         `ESP_OK` on success
 */
esp_err_t scd4x_get_temperature_offset_ticks(i2c_dev_t *dev, uint16_t *t_offset);

/**
 * @brief Get temperature offset in °C.
 *
 * See ::scd4x_get_temperature_offset_ticks() for more details.
 *
 * @note Only available in idle mode.
 *
 * @param dev      Device descriptor
 * @param t_offset Temperature offset in degrees Celsius (°C)
 * @return         `ESP_OK` on success
 */
esp_err_t scd4x_get_temperature_offset(i2c_dev_t *dev, float *t_offset);

/**
 * @brief Set temperature offset in ticks.
 *
 * Setting the temperature offset of the SCD4x inside the customer device
 * correctly allows the user to leverage the RH and T output signal. Note
 * that the temperature offset can depend on various factors such as the
 * SCD4x measurement mode, self-heating of close components, the ambient
 * temperature and air flow. Thus, the SCD4x temperature offset should be
 * determined inside the customer device under its typical operation and in
 * thermal equilibrium.
 *
 * @note Only available in idle mode.
 *
 * @param dev      Device descriptor
 * @param t_offset Temperature offset.
 *                 Convert °C to value by: T * 2^16 / 175
 * @return         `ESP_OK` on success
 */
esp_err_t scd4x_set_temperature_offset_ticks(i2c_dev_t *dev, uint16_t t_offset);

/**
 * @brief Set temperature offset in °C.
 *
 * See ::scd4x_set_temperature_offset_ticks() for more details.
 *
 * @note Only available in idle mode.
 *
 * @param dev      Device descriptor
 * @param t_offset Temperature offset in degrees Celsius (°C)
 * @return         `ESP_OK` on success
 */
esp_err_t scd4x_set_temperature_offset(i2c_dev_t *dev, float t_offset);

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
esp_err_t scd4x_get_sensor_altitude(i2c_dev_t *dev, uint16_t *altitude);

/**
 * @brief Set sensor altitude in meters above sea level.
 *
 * Note that setting a sensor altitude to the sensor overrides any pressure
 * compensation based on a previously set ambient pressure.
 *
 * @note Only available in idle mode.
 *
 * @param dev      Device descriptor.
 * @param altitude Sensor altitude in meters.
 * @return         `ESP_OK` on success
 */
esp_err_t scd4x_set_sensor_altitude(i2c_dev_t *dev, uint16_t altitude);

/**
 * @brief Set ambient pressure.
 *
 * The set_ambient_pressure command can be sent during periodic measurements
 * to enable continuous pressure compensation. Note that setting an ambient
 * pressure to the sensor overrides any pressure compensation based on a
 * previously set sensor altitude.
 *
 * @note Available during measurements.
 *
 * @param dev      Device descriptor.
 * @param pressure Ambient pressure in hPa.
 *                 Convert value to Pa by: value * 100
 * @return         `ESP_OK` on success
 */
esp_err_t scd4x_set_ambient_ressure(i2c_dev_t *dev, uint16_t pressure);

/**
 * @brief Perform forced recalibration.
 *
 * To successfully conduct an accurate forced recalibration, the following
 * steps need to be carried out:
 * - Operate the SCD4x in a periodic measurement mode for > 3 minutes in an
 *   environment with homogenous and constant CO₂ concentration.
 * - Stop periodic measurement. Wait 500 ms.
 * - Subsequently call scd4x_perform_forced_recalibration() and optionally
 *   read out the baseline correction. A return value of 0xffff indicates
 *   that the forced recalibration failed.
 *
 * @param dev                      Device descriptor.
 * @param target_co2_concentration Target CO₂ concentration in ppm.
 * @param frc_correction           FRC correction value in CO₂ ppm or 0xFFFF
 *                                 if the command failed.
 * @return                         `ESP_OK` on success
 */
esp_err_t scd4x_perform_forced_recalibration(i2c_dev_t *dev,
        uint16_t target_co2_concentration, uint16_t *frc_correction);

/**
 * @brief Get automatic self calibration (ASC) state.
 *
 * By default, the ASC is enabled.
 *
 * @param dev     Device descriptor.
 * @param enabled true if ASC is enabled, false otherwise
 * @return        `ESP_OK` on success
 */
esp_err_t scd4x_get_automatic_self_calibration(i2c_dev_t *dev, bool *enabled);

/**
 * @brief Enable or disable automatic self calibration (ASC).
 *
 * By default, the ASC is enabled.
 *
 * @param dev     Device descriptor.
 * @param enabled true to enable ASC, false to disable ASC
 * @return        `ESP_OK` on success
 */
esp_err_t scd4x_set_automatic_self_calibration(i2c_dev_t *dev, bool enabled);

/**
 * @brief Start low power periodic measurement.
 *
 * Signal update interval is 30 seconds.
 *
 * @note This command is only available in idle mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_start_low_power_periodic_measurement(i2c_dev_t *dev);

/**
 * @brief Check whether new measurement data is available for read-out.
 *
 * @param dev        Device descriptor
 * @param data_ready true if data is ready, false otherwise
 * @return           `ESP_OK` on success
 */
esp_err_t scd4x_get_data_ready_status(i2c_dev_t *dev, bool *data_ready);

/**
 * @brief Store current configuration in EEPROM.
 *
 * Configuration settings such as the temperature offset, sensor altitude and
 * the ASC enabled/disabled parameter are by default stored in the volatile
 * memory (RAM) only and will be lost after a power-cycle. This funciton
 * stores the current configuration in the EEPROM of the SCD4x, making them
 * resistant to power-cycling. To avoid unnecessary wear of the EEPROM,
 * function should only be called when persistence is required and if actual
 * changes to the configuration have been made. Note that field calibration
 * history (i.e. FRC and ASC) is stored in the EEPROM automatically.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_persist_settings(i2c_dev_t *dev);

/**
 * @brief Read serial number.
 *
 * Reading out the serial number can be used to identify the chip and to verify
 * the presence of the sensor. Function returns 3 words. Together, the 3 words
 * constitute a unique serial number with a length of 48 bits (big endian format).
 *
 * @param dev     Device descriptor
 * @param serial0 First word of the 48 bit serial number
 * @param serial1 Second word of the 48 bit serial number
 * @param serial2 Third word of the 48 bit serial number
 * @return        `ESP_OK` on success
 */
esp_err_t scd4x_get_serial_number(i2c_dev_t *dev, uint16_t *serial0, uint16_t *serial1, uint16_t *serial2);

/**
 * @brief Perform self-test.
 *
 * This function can be used as an  end-of-line test to confirm sensor
 * functionality.
 *
 * @param dev         Device descriptor
 * @param malfunction true if malfunction detected, false if device is OK
 * @return            `ESP_OK` on success
 */
esp_err_t scd4x_perform_self_test(i2c_dev_t *dev, bool *malfunction);

/**
 * @brief Factory reset sensor.
 *
 * Initiates the reset of all configurations stored in the EEPROM and erases the
 * FRC and ASC algorithm history.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_perform_factory_reset(i2c_dev_t *dev);

/**
 * @brief Reinitialize sensor.
 *
 * The reinit command reinitializes the sensor by reloading user settings from
 * EEPROM. Before sending the reinit command, the stop measurement command must
 * be issued. If reinit command does not trigger the desired re-initialization,
 * a power-cycle should be applied to the SCD4x.
 *
 * @note Only available in idle mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_reinit(i2c_dev_t *dev);

/**
 * @brief Perform single measurement.
 *
 * On-demand measurement of CO₂ concentration, relative humidity and temperature.
 * The sensor output is read with the ::scd4x_read_measurement() function.
 *
 * @note Only available in idle mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_measure_single_shot(i2c_dev_t *dev);

/**
 * @brief Perform single measurement of of relative humidity and temperature
 *        only.
 *
 * @note Only available in idle mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_measure_single_shot_rht_only(i2c_dev_t *dev);

/**
 * @brief Put the sensor from idle to sleep mode.
 *
 * Call this function to reduce current consumption.
 *
 * @note Only available in idle mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_power_down(i2c_dev_t *dev);

/**
 * @brief Wake up sensor from sleep mode to idle mode.
 *
 * @note Only available in sleep mode.
 *
 * @param dev Device descriptor
 * @return    `ESP_OK` on success
 */
esp_err_t scd4x_wake_up(i2c_dev_t *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SCD4X_H__ */

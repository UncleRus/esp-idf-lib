/*
 * Copyright (c) 2022 Jose Manuel Perez <user@your.dom.ain>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file lc709203f.h
 * @defgroup lc709203f lc709203f
 * @{
 *
 * LC709203F Battery Fuel Gauge driver
 *
 */

#ifndef __LC709203F__H__
#define __LC709203F__H__

#include <esp_err.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LC709203F_REG_BEFORE_RSOC       0x04 ///< Initialize before RSOC
#define LC709203F_REG_THERMISTOR_B      0x06 ///< Read/write thermistor B
#define LC709203F_REG_INITIAL_RSOC      0x07 ///< Initialize RSOC calculation
#define LC709203F_REG_CELL_TEMPERATURE  0x08 ///< Read/write batt temperature
#define LC709203F_REG_CELL_VOLTAGE      0x09 ///< Read batt voltage
#define LC709203F_REG_CURRENT_DIRECTION 0x0A ///< Read/write current direction
#define LC709203F_REG_APA               0x0B ///< Read/write Adjustment Pack Application
#define LC709203F_REG_APT               0x0C ///< Read/write Adjustment Pack Thermistor
#define LC709203F_REG_RSOC              0x0D ///< Read state of charge
#define LC709203F_REG_CELL_ITE          0x0F ///< Read batt indicator to empty
#define LC709203F_REG_IC_VERSION        0x11 ///< Read IC version
#define LC709203F_REG_CHANGE_PARAMETER  0x12 ///< Set the battery profile
#define LC709203F_REG_ALARM_LOW_RSOC    0x13 ///< Alarm on percent threshold
#define LC709203F_REG_ALARM_LOW_VOLTAGE 0x14 ///< Alarm on voltage threshold
#define LC709203F_REG_IC_POWER_MODE     0x15 ///< Sets sleep/power mode
#define LC709203F_REG_STATUS_BIT        0x16 ///< Temperature obtaining method
#define LC709203F_REG_NUM_PARAMETER     0x1A ///< Batt profile code

/*!  Battery temperature source */
typedef enum {
    LC709203F_TEMP_MODE_I2C = 0x0000,
    LC709203F_TEMP_MODE_THERMISTOR = 0x0001,
} lc709203f_temp_mode_t;

/*!  Chip power state */
typedef enum {
    LC709203F_POWER_MODE_OPERATIONAL = 0x0001,
    LC709203F_POWER_MODE_SLEEP = 0x0002,
} lc709203f_power_mode_t;

typedef enum {
    LC709203F_DIRECTION_AUTO = 0x0000,
    LC709203F_DIRECTION_CHARGE = 0x0001,
    LC709203F_DIRECTION_DISCHARGE = 0xFFFF,
} lc709203f_direction_t;

typedef enum {
    LC709203F_BATTERY_PROFILE_0 = 0x0000,
    LC709203F_BATTERY_PROFILE_1 = 0x0001,
} lc709203f_battery_profile_t;

/**
 * @brief Initialize device descriptor
 *
 * @param[in] dev Device descriptor
 * @param[in] port I2C port number
 * @param[in] sda_gpio GPIO pin number for SDA
 * @param[in] scl_gpio GPIO pin number for SCL
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param[in] dev Device descriptor
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_free_desc(i2c_dev_t *dev);

/**
 * @brief Executes RSOC initialization with sampled maximum voltage when 0xAA55 is set.
 *
 * @param[in] dev Device descriptor
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_before_rsoc(i2c_dev_t *dev);

/**
 * @brief Executes RSOC initialization when 0xAA55 is set.
 *
 * @param[in] dev Device descriptor
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_initial_rsoc(i2c_dev_t *dev);

/**
 * @brief Get alarm low rsoc (% unit)
 *
 * @note Disable alarm setting RSOC as 0 (0x0000)
 *       Enable alarm setting RSOC in
 *
 * @param[in] dev Device descriptor
 * @param[out] rsoc RSOC value (%)
 * @return
 *      `ESP_INVALID_ARG` null dev or rsoc out of range 0-100
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_alarm_low_rsoc(i2c_dev_t *dev, uint8_t *rsoc);

/**
 * @brief Get alarm low voltage (mV unit)
 *
 * @param[in] dev Device descriptor
 * @param[out] voltage Voltage value (mV)
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_alarm_low_voltage(i2c_dev_t *dev, uint16_t *voltage);

/**
 * @brief Get APA (adjustment pack application)
 *
 * @param[in] dev Device descriptor
 * @param[out] apa Current APA value
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_apa(i2c_dev_t *dev, uint8_t *apa);

/**
 * @brief Get APT (adjustment pack thermistor)
 *
 * @param[in] dev Device descriptor
 * @param[out] apt Current APT value
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_apt(i2c_dev_t *dev, uint16_t *apt);

/**
 * @brief Get battery profile
 *
 * @param[in] dev Device descriptor
 * @param[out] profile Current profile
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_battery_profile(i2c_dev_t *dev, lc709203f_battery_profile_t *profile);

/**
 * @brief Get battery profile code
 *
 * @param[in] dev Device descriptor
 * @param[out] code Current profile code. 0x3001 or 0x0504 only.
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_battery_profile_code(i2c_dev_t *dev, uint16_t *code);

/**
 * @brief Get ITE (indicator to empty) "RSOC (%) on 0-1000 scale"
 *
 * @param[in] dev Device descriptor
 * @param[out] ite Current RSOC value
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_cell_ite(i2c_dev_t *dev, uint16_t *ite);

/**
 * @brief Get cell temperature in ºK. Min. unit is 0.1K
 *
 * @note Conversion ºK -> ºC = ºK + 273
 *       Conversion ºC -> ºK = ºK + 273
 *       Typical values:
 *          0ºC -> 0x0AAC
 *          25ºC -> 0x0BA6 (default)
 *
 * @param[in] dev Device descriptor
 * @param[out] temperature Current temperature
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_cell_temperature(i2c_dev_t *dev, float *temperature);

/**
 * @brief Get cell temperature in ºC.
 *
 * @param[in] dev Device descriptor
 * @param[out] temperature Current temperature
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_cell_temperature_celsius(i2c_dev_t *dev, float *temperature);

/**
 * @brief Get cell voltage
 *
 * @param[in] dev Device descriptor
 * @param[out] voltage Current voltage
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_cell_voltage(i2c_dev_t *dev, uint16_t *voltage);

/**
 * @brief Set current direction
 *
 * @param[in] dev Device descriptor
 * @param[out] direction Current direction
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_direction(i2c_dev_t *dev, lc709203f_direction_t *direction);

/**
 * @brief Get ID number of an IC
 *
 * @param[in] dev Device descriptor
 * @param[out] ic_version IC ID number
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_ic_version(i2c_dev_t *dev, uint16_t *ic_version);

/**
 * @brief Get power mode
 *
 * @param[in] dev Device descriptor
 * @param[out] mode Power mode
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_power_mode(i2c_dev_t *dev, lc709203f_power_mode_t *mode);

/**
 * @brief Get RSOC (%) on 0-100 scale
 *
 * @param[in] dev Device descriptor
 * @param[out] rsoc Current RSOC value
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_rsoc(i2c_dev_t *dev, uint16_t *rsoc);

/**
 * @brief Get temperature mode
 *
 * @param[in] dev Device descriptor
 * @param[out] mode Temperature obtaining mode
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_temp_mode(i2c_dev_t *dev, lc709203f_temp_mode_t *mode);

/**
 * @brief Get B-constant of the thermistor to be measured
 *
 * @param[in] dev Device descriptor
 * @param[out] value B-constant value
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_get_thermistor_b(i2c_dev_t *dev, uint16_t *value);

/**
 * @brief Set alarm low rsoc (% unit)
 *
 * @note Disable alarm setting RSOC as 0 (0x0000)
 *       Enable alarm setting RSOC in
 *
 * @param[in] dev Device descriptor
 * @param[in] rsoc RSOC value (%)
 * @return
 *      `ESP_INVALID_ARG` null dev or rsoc out of range 0-100
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_alarm_low_rsoc(i2c_dev_t *dev, uint8_t rsoc);

/**
 * @brief Set alarm low voltage (mV unit)
 *
 * @param[in] dev Device descriptor
 * @param[in] voltage Voltage value (mV)
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_alarm_low_voltage(i2c_dev_t *dev, uint16_t voltage);

/**
 * @brief Set APA (adjustment pack application)
 *
 * @param[in] dev Device descriptor
 * @param[in] apa Value to set
 * @return `ESP_OK` on success
 */
esp_err_t lc709203f_set_apa(i2c_dev_t *dev, uint8_t apa);

/**
 * @brief Set APA (adjustment pack thermistor)
 *
 * @param[in] dev Device descriptor
 * @param[in] apt Value to set
 * @return
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_apt(i2c_dev_t *dev, uint16_t apt);

/**
 * @brief Set battery profile
 *
 * @param[in] dev Device descriptor
 * @param[in] profile Current profile
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_battery_profile(i2c_dev_t *dev, lc709203f_battery_profile_t profile);

/**
 * @brief Set cell temperature in ºK. Min. unit is 0.1ºK
 *
 * @note Conversion ºK -> ºC = ºK - 273
 *       Conversion ºC -> ºK = ºK + 273
 *       Typical values:
 *          0ºC -> 0x0AAC
 *          25ºC -> 0x0BA6 (default)
 *
 * @param[in] dev Device descriptor
 * @param[in] temperature Temperature to set
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_cell_temperature(i2c_dev_t *dev, float temperature);

/**
 * @brief Set cell temperature in ºC.
 *
 * @param[in] dev Device descriptor
 * @param[in] temperature Temperature to set
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_cell_temperature_celsius(i2c_dev_t *dev, float temperature);

/**
 * @brief Set current direction
 *
 * @param[in] dev Device descriptor
 * @param[in] direction Current direction
 * @return
 *      `ESP_INVALID_ARG` null dev. In I2C mode, temperature not in range 0x09E4-0x0D04 ()
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_current_direction(i2c_dev_t *dev, lc709203f_direction_t direction);

/**
 * @brief Set power mode
 *
 * @param[in] dev Device descriptor
 * @param[in] mode Power mode
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_power_mode(i2c_dev_t *dev, lc709203f_power_mode_t mode);

/**
 * @brief Set temperature mode
 *
 * @param[in] dev Device descriptor
 * @param[in] mode Temperature obtaining mode
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_temp_mode(i2c_dev_t *dev, lc709203f_temp_mode_t mode);

/**
 * @brief Set B-constant of ther thermistor to be measured
 *
 * @param[in] dev Device descriptor
 * @param[in] value B-constant value
 * @return
 *      `ESP_INVALID_ARG` null dev
 *      `ESP_OK` on success
 */
esp_err_t lc709203f_set_thermistor_b(i2c_dev_t *dev, uint16_t value);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __LC709203F__H__

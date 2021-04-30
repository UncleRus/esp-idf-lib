/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file mcp9808.h
 * @defgroup mcp9808 mcp9808
 * @{
 *
 * ESP-IDF driver for MCP9808 Digital Temperature Sensor
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP9808_H__
#define __MCP9808_H__

#include <stdbool.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP9808_I2C_ADDR_000 0x18 //!< I2C address, pins: A0=0, A1=0, A2=0
#define MCP9808_I2C_ADDR_001 0x19 //!< I2C address, pins: A0=1, A1=0, A2=0
#define MCP9808_I2C_ADDR_010 0x1A //!< I2C address, pins: A0=0, A1=1, A2=0
#define MCP9808_I2C_ADDR_011 0x1B //!< I2C address, pins: A0=1, A1=1, A2=0
#define MCP9808_I2C_ADDR_100 0x1C //!< I2C address, pins: A0=0, A1=0, A2=1
#define MCP9808_I2C_ADDR_101 0x1D //!< I2C address, pins: A0=1, A1=0, A2=1
#define MCP9808_I2C_ADDR_110 0x1E //!< I2C address, pins: A0=0, A1=1, A2=1
#define MCP9808_I2C_ADDR_111 0x1F //!< I2C address, pins: A0=1, A1=1, A2=1

/**
 * Device mode
 */
typedef enum {
    MCP9808_CONTINUOUS = 0, //!< Continuous measurement mode, default
    MCP9808_SHUTDOWN        //!< Shutdown mode
} mcp9808_mode_t;

/**
 * T upper and T lower hysteresis
 */
typedef enum {
    MCP9808_HYST_0 = 0, //!< 0.0 deg.C, default
    MCP9808_HYST_1_5,   //!< 1.5 deg.C
    MCP9808_HYST_3,     //!< 3.0 deg.C
    MCP9808_HYST_6      //!< 6.0 deg.C
} mcp9808_hysteresis_t;

/**
 * Alert output mode
 */
typedef enum {
    MCP9808_ALERT_DISABLED = 0, //!< Alert output disabled, default
    MCP9808_ALERT_COMPARATOR,   //!< Alert output in comparator mode
    MCP9808_ALERT_INTERRUPT     //!< Alert output in interrupt mode
} mcp9808_alert_mode_t;

/**
 * Alert output select
 */
typedef enum {
    MCP9808_ALERT_UP_LOW_CRIT = 0, //!< Alert when T > T upper or T < T lower or T > T crit, default
    MCP9808_ALERT_CRIT,            //!< Alert when T > T crit
} mcp9808_alert_select_t;

/**
 * Alert output polarity
 */
typedef enum {
    MCP9808_ALERT_LOW = 0, //!< Active-low, pull-up resistor required, default
    MCP9808_ALERT_HIGH,    //!< Active-high
} mcp9808_alert_polarity_t;

/**
 * Temperature resolution
 */
typedef enum {
    MCP9808_RES_0_5 = 0, //!< Resolution = +0.5 deg.C, conversion time = 30 ms, typical sample rate = 33 Hz
    MCP9808_RES_0_25,    //!< Resolution = +0.25 deg.C, conversion time = 65 ms, typical sample rate = 15 Hz
    MCP9808_RES_0_125,   //!< Resolution = +0.125 deg.C, conversion time = 130 ms, typical sample rate = 7 Hz
    MCP9808_RES_0_0625   //!< Resolution = +0.0625 deg.C, conversion time = 250 ms, typical sample rate = 4 Hz, default
} mcp9808_resolution_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_free_desc(i2c_dev_t *dev);

/**
 * @brief Init device
 *
 * Set device configuration to default, clear lock bits
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_init(i2c_dev_t *dev);

/**
 * @brief Set device mode
 *
 * @param dev Device descriptor
 * @param mode Power mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_set_mode(i2c_dev_t *dev, mcp9808_mode_t mode);

/**
 * @brief Get device mode
 *
 * @param dev Device descriptor
 * @param[out] mode Current power mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_get_mode(i2c_dev_t *dev, mcp9808_mode_t *mode);

/**
 * @brief Set temperature resolution
 *
 * @param dev Device descriptor
 * @param res Resolution
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_set_resolution(i2c_dev_t *dev, mcp9808_resolution_t res);

/**
 * @brief Get temperature resolution
 *
 * @param dev Device descriptor
 * @param[out] res Resolution
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_get_resolution(i2c_dev_t *dev, mcp9808_resolution_t *res);

/**
 * @brief Configure alert parameters
 *
 * @param dev Device descriptor
 * @param mode Alert mode
 * @param sel Alert window (see datasheet)
 * @param polarity Alert output polarity
 * @param hyst Alert limits hysteresis
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_set_alert_config(i2c_dev_t *dev, mcp9808_alert_mode_t mode,
        mcp9808_alert_select_t sel, mcp9808_alert_polarity_t polarity, mcp9808_hysteresis_t hyst);

/**
 * @brief Get alert configuration
 *
 * @param dev Device descriptor
 * @param[out] mode Alert mode
 * @param[out] sel Alert window (see datasheet)
 * @param[out] polarity Alert output polarity
 * @param[out] hyst Alert limits hysteresis
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_get_alert_config(i2c_dev_t *dev, mcp9808_alert_mode_t *mode,
        mcp9808_alert_select_t *sel, mcp9808_alert_polarity_t *polarity, mcp9808_hysteresis_t *hyst);

/**
 * @brief Set alert temperature limits
 *
 * @param dev Device descriptor
 * @param t_upper Upper temperature
 * @param t_lower Lower temperature
 * @param t_crit  Critical temperature
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_set_limits(i2c_dev_t *dev, float t_upper, float t_lower, float t_crit);

/**
 * @brief Get alert temperature limits
 *
 * @param dev Device descriptor
 * @param[out] t_upper Upper temperature
 * @param[out] t_lower Lower temperature
 * @param[out] t_crit  Critical temperature
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_get_limits(i2c_dev_t *dev, float *t_upper, float *t_lower, float *t_crit);

/**
 * @brief Set alert status
 *
 * @param dev Device descriptor
 * @param alert True for alert
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_set_alert_status(i2c_dev_t *dev, bool alert);

/**
 * @brief Get current alert status
 *
 * @param dev Device descriptor
 * @param[out] alert Alert status
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_get_alert_status(i2c_dev_t *dev, bool *alert);

/**
 * @brief Clear interrupt bit
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_clear_interrupt(i2c_dev_t *dev);

/**
 * @brief Read temperature
 *
 * @param dev Device descriptor
 * @param[out] t Ambient temperature
 * @param[out] lower True if T a < T lower, can be NULL
 * @param[out] upper True if T a > T upper, can be NULL
 * @param[out] crit True if T a >= T critical, can be NULL
 * @return `ESP_OK` on success
 */
esp_err_t mcp9808_get_temperature(i2c_dev_t *dev, float *t, bool *lower, bool *upper, bool *crit);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MCP9808_H__ */

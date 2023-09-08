/*
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file mcp4725.h
 * @defgroup mcp4725 mcp4725
 * @{
 *
 * ESP-IDF Driver for 12-bit DAC MCP4725
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP4725_H__
#define __MCP4725_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP4725A0_I2C_ADDR0 0x60
#define MCP4725A0_I2C_ADDR1 0x61
#define MCP4725A1_I2C_ADDR0 0x62
#define MCP4725A1_I2C_ADDR1 0x63
#define MCP4725A2_I2C_ADDR0 0x64
#define MCP4725A2_I2C_ADDR1 0x65

#define MCP4725_MAX_VALUE 0x0fff

/**
 * Power mode, see datasheet
 */
typedef enum
{
    MCP4725_PM_NORMAL = 0,   //!< Normal mode
    MCP4725_PM_PD_1K,        //!< Power down, 1kOhm resistor to ground
    MCP4725_PM_PD_100K,      //!< Power down, 100kOhm resistor to ground
    MCP4725_PM_PD_500K,      //!< Power down, 500kOhm resistor to ground
} mcp4725_power_mode_t;

/**
 * @brief Initialize device descriptor
 *
 * Default SCL frequency is 1MHz
 *
 * @param dev I2C device descriptor
 * @param port I2C port number
 * @param addr I2C address,
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_free_desc(i2c_dev_t *dev);

/**
 * @brief Get device EEPROM status
 *
 * @param dev I2C device descriptor
 * @param busy true when EEPROM is busy
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_eeprom_busy(i2c_dev_t *dev, bool *busy);

/**
 * @brief Get power mode
 *
 * @param dev I2C device descriptor
 * @param eeprom Read power mode from EEPROM if true
 * @param[out] mode Power mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_get_power_mode(i2c_dev_t *dev, bool eeprom, mcp4725_power_mode_t *mode);

/**
 * @brief Set power mode
 *
 * @param dev I2C device descriptor
 * @param mode Power mode
 * @param eeprom Store mode to device EEPROM if true
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_set_power_mode(i2c_dev_t *dev, bool eeprom, mcp4725_power_mode_t mode);

/**
 * @brief Get current DAC value
 *
 * @param dev I2C device descriptor
 * @param eeprom Read value from device EEPROM if true
 * @param[out] value Raw output value, 0..4095
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_get_raw_output(i2c_dev_t *dev, bool eeprom, uint16_t *value);

/**
 * @brief Set DAC output value
 *
 * @param dev I2C device descriptor
 * @param value Raw output value, 0..4095
 * @param eeprom Store value to device EEPROM if true
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_set_raw_output(i2c_dev_t *dev, uint16_t value, bool eeprom);

/**
 * @brief Get current DAC output voltage
 *
 * @param dev I2C device descriptor
 * @param vdd Device operating voltage, volts
 * @param eeprom Read voltage from device EEPROM if true
 * @param[out] voltage Current output voltage, volts
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_get_voltage(i2c_dev_t *dev, float vdd, bool eeprom, float *voltage);

/**
 * @brief Set DAC output voltage
 *
 * @param dev I2C device descriptor
 * @param vdd Device operating voltage, volts
 * @param value Output value, volts
 * @param eeprom Store value to device EEPROM if true
 * @return `ESP_OK` on success
 */
esp_err_t mcp4725_set_voltage(i2c_dev_t *dev, float vdd, float value, bool eeprom);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MCP4725_H__ */

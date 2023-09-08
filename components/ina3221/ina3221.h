/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Zaltora <https://github.com/Zaltora>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ina3221.h
 * @defgroup ina3221 ina3221
 * @{
 *
 * ESP-IDF driver for Shunt and Bus Voltage Monitor INA3221
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Zaltora <https://github.com/Zaltora>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __INA3221_H__
#define __INA3221_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INA3221_I2C_ADDR_GND 0x40 ///< A0 to GND
#define INA3221_I2C_ADDR_VS  0x41 ///< A0 to Vs+
#define INA3221_I2C_ADDR_SDA 0x42 ///< A0 to SDA
#define INA3221_I2C_ADDR_SCL 0x43 ///< A0 to SCL

#define INA3221_BUS_NUMBER 3  ///< Number of shunt available

/**
 *  Default register values after reset
 */
#define INA3221_DEFAULT_CONFIG                   (0x7127)
#define INA3221_DEFAULT_MASK                     (0x0002)
#define INA3221_DEFAULT_POWER_UPPER_LIMIT        (0x2710) //10V
#define INA3221_DEFAULT_POWER_LOWER_LIMIT        (0x2328) //9V

#define INA3221_MASK_CONFIG (0x7C00)

/**
 * Number of samples
 */
typedef enum
{
    INA3221_AVG_1 = 0,  ///< Default
    INA3221_AVG_4,
    INA3221_AVG_16,
    INA3221_AVG_64,
    INA3221_AVG_128,
    INA3221_AVG_256,
    INA3221_AVG_512,
    INA3221_AVG_1024,
} ina3221_avg_t;

/**
 * Channel selection list
 */
typedef enum
{
    INA3221_CHANNEL_1 = 0,
    INA3221_CHANNEL_2,
    INA3221_CHANNEL_3,
} ina3221_channel_t;

/**
 * Conversion time in us
 */
typedef enum
{
    INA3221_CT_140 = 0,
    INA3221_CT_204,
    INA3221_CT_332,
    INA3221_CT_588,
    INA3221_CT_1100,  ///< Default
    INA3221_CT_2116,
    INA3221_CT_4156,
    INA3221_CT_8244,
} ina3221_ct_t;

/**
 * Config description register
 */
typedef union
{
    struct
    {
        uint16_t esht :1; ///< Enable/Disable shunt measure    // LSB
        uint16_t ebus :1; ///< Enable/Disable bus measure
        uint16_t mode :1; ///< Single shot measure or continuous mode
        uint16_t vsht :3; ///< Shunt voltage conversion time
        uint16_t vbus :3; ///< Bus voltage conversion time
        uint16_t avg  :3; ///< number of sample collected and averaged together
        uint16_t ch3  :1; ///< Enable/Disable channel 3
        uint16_t ch2  :1; ///< Enable/Disable channel 2
        uint16_t ch1  :1; ///< Enable/Disable channel 1
        uint16_t rst  :1; ///< Set this bit to 1 to reset device  // MSB
    };
    uint16_t config_register;
} ina3221_config_t;

/**
 * Mask/enable description register
 */
typedef union
{
    struct
    {
        uint16_t cvrf :1; ///< Conversion ready flag (1: ready)   // LSB
        uint16_t tcf  :1; ///< Timing control flag
        uint16_t pvf  :1; ///< Power valid flag
        uint16_t wf   :3; ///< Warning alert flag (Read mask to clear) (order : Channel1:channel2:channel3)
        uint16_t sf   :1; ///< Sum alert flag (Read mask to clear)
        uint16_t cf   :3; ///< Critical alert flag (Read mask to clear) (order : Channel1:channel2:channel3)
        uint16_t cen  :1; ///< Critical alert latch (1:enable)
        uint16_t wen  :1; ///< Warning alert latch (1:enable)
        uint16_t scc3 :1; ///< channel 3 sum (1:enable)
        uint16_t scc2 :1; ///< channel 2 sum (1:enable)
        uint16_t scc1 :1; ///< channel 1 sum (1:enable)
        uint16_t      :1; ///< Reserved         //MSB
    };
    uint16_t mask_register;
} ina3221_mask_t;

/**
 *  Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;                  ///< I2C device descriptor
    uint16_t shunt[INA3221_BUS_NUMBER]; ///< Memory of shunt value (mOhm)
    ina3221_config_t config;                  ///< Memory of ina3221 config
    ina3221_mask_t mask;                      ///< Memory of mask_config
} ina3221_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_init_desc(ina3221_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_free_desc(ina3221_t *dev);

/**
 * @brief Write current config to device
 *
 * Sync internal config buffer and mask with external device register. (When struct is set manually).
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_sync(ina3221_t *dev);

/**
 * @brief Trigger measurement
 *
 * Send current config register to trig a measurement in single-shot mode.
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_trigger(ina3221_t *dev);

/**
 * @brief Read status from device
 *
 * Get mask register from the device, used to read flags.
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_get_status(ina3221_t *dev);

/**
 * @brief Set options for bus and shunt
 *
 * @param dev Device descriptor
 * @param mode Selection of measurement (true : continuous // false : single-shot)
 * @param bus Enable/Disable bus measures
 * @param shunt Enable/Disable shunt measures
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_options(ina3221_t *dev, bool mode, bool bus, bool shunt);

/**
 * @brief Select channels
 *
 * @param dev Device descriptor
 * @param ch1 Enable/Disable channel 1 (true : enable // false : disable)
 * @param ch2 Enable/Disable channel 2 (true : enable // false : disable)
 * @param ch3 Enable/Disable channel 3 (true : enable // false : disable)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_enable_channel(ina3221_t *dev, bool ch1, bool ch2, bool ch3);

/**
 * @brief Select channel to be sum (don't impact enable channel status)
 *
 * @param dev Device descriptor
 * @param ch1 Enable/Disable channel 1 (true : enable // false : disable)
 * @param ch2 Enable/Disable channel 2 (true : enable // false : disable)
 * @param ch3 Enable/Disable channel 3 (true : enable // false : disable)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_enable_channel_sum(ina3221_t *dev, bool ch1, bool ch2, bool ch3);

/**
 * @brief enable/disable latch on warning and critical alert pin
 *
 * @param dev Device descriptor
 * @param warning Enable/Disable warning latch (true : Latch // false : Transparent)
 * @param critical Enable/Disable critical latch (true : Latch // false : Transparent)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_enable_latch_pin(ina3221_t *dev, bool warning, bool critical);

/**
 * @brief Set average (number of samples measured)
 *
 * @param dev Device descriptor
 * @param avg Value of average selection
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_average(ina3221_t *dev, ina3221_avg_t avg);

/**
 * @brief Set conversion time for bus.
 *
 * @param dev Device descriptor
 * @param ct Value of conversion time selection
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_bus_conversion_time(ina3221_t *dev, ina3221_ct_t ct);

/**
 * @brief Set conversion time for shunt.
 *
 * @param dev Device descriptor
 * @param ct Value of conversion time selection
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_shunt_conversion_time(ina3221_t *dev, ina3221_ct_t ct);

/**
 * @brief Reset device
 *
 * Device will be configured like POR (Power-On-Reset)
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_reset(ina3221_t *dev);

/**
 * @brief Get Bus voltage (V)
 *
 * @param dev Device descriptor
 * @param channel Select channel value to get
 * @param voltage Data pointer to get bus voltage (V)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_get_bus_voltage(ina3221_t *dev, ina3221_channel_t channel, float *voltage);

/**
 * @brief Get Shunt voltage (mV) and current (mA)
 *
 * @param dev Device descriptor
 * @param channel Select channel value to get
 * @param voltage Data pointer to get shunt voltage (mV)
 * @param current Data pointer to get shunt voltage (mA)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_get_shunt_value(ina3221_t *dev, ina3221_channel_t channel, float *voltage, float *current);

/**
 * @brief Get Shunt-voltage (mV) sum value of selected channels
 *
 * @param dev Device descriptor
 * @param voltage Data pointer to get shunt voltage (mV)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_get_sum_shunt_value(ina3221_t *dev, float *voltage);

/**
 * @brief Set Critical alert
 *
 * Alert when measurement(s) is greater
 *
 * @param dev Device descriptor
 * @param channel Select channel value to set
 * @param current Value to set (mA) // max : 163800/shunt (mOhm)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_critical_alert(ina3221_t *dev, ina3221_channel_t channel, float current);

/**
 * @brief Set Warning alert
 *
 * Alert when average measurement(s) is greater
 *
 * @param dev Device descriptor
 * @param channel Select channel value to set
 * @param current Value to set (mA)  // max : 163800/shunt (mOhm)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_warning_alert(ina3221_t *dev, ina3221_channel_t channel, float current);

/**
 * @brief Set Sum Warning alert
 *
 * Compared to each completed cycle of all selected channels : Sum register
 *
 * @param dev Device descriptor
 * @param voltage voltage to set (mV) //  max : 655.32
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_sum_warning_alert(ina3221_t *dev, float voltage);

/**
 * @brief Set Power-valid upper-limit
 *
 * Used to determine if power conditions are met. Bus needs to be enabled.
 * If bus voltage exceed the value set, PV pin will be set high.
 *
 * @param dev Device descriptor
 * @param voltage voltage to set (V)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_power_valid_upper_limit(ina3221_t *dev, float voltage);

/**
 * @brief Set Power-valid lower-limit
 *
 * Used to determine if power conditions are met. Bus needs to be enabled.
 * If bus voltage drops below the value set, PV pin will be set low.
 *
 * @param dev Device descriptor
 * @param voltage Voltage to set (V)
 * @return ESP_OK to indicate success
 */
esp_err_t ina3221_set_power_valid_lower_limit(ina3221_t *dev, float voltage);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __INA3221_H__ */

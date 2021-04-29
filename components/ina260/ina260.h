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
 * @file ina260.h
 * @defgroup ina260 ina260
 * @{
 *
 * ESP-IDF driver for INA260 precision digital current and power monitor
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 *
 * @todo Add support for SPI interface
 */
#ifndef __INA260_H__
#define __INA260_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INA260_ADDR_PIN_GND 0x00
#define INA260_ADDR_PIN_VS  0x01
#define INA260_ADDR_PIN_SDA 0x02
#define INA260_ADDR_PIN_SCL 0x03

/**
 * Macro to define I2C address
 *
 * Examples:
 *    INA260_ADDR(INA260_ADDR_PIN_GND, INA260_ADDR_PIN_GND) = 0x40 (A0 = A1 = GND)
 *    INA260_ADDR(INA260_ADDR_PIN_VS, INA260_ADDR_PIN_SDA)  = 0x49 (A0 = VS, A1 = SDA)
 */
#define INA260_ADDR(A0, A1) (0x40 | ((A1) << 2) | (A0))

/**
 * Averaging mode.
 * Determines the number of samples that are collected and averaged.
 */
typedef enum
{
    INA260_AVG_1 = 0, //!< 1 sample, default
    INA260_AVG_4,     //!< 4 samples
    INA260_AVG_16,    //!< 16 samples
    INA260_AVG_64,    //!< 64 samples
    INA260_AVG_128,   //!< 128 samples
    INA260_AVG_256,   //!< 256 samples
    INA260_AVG_512,   //!< 512 samples
    INA260_AVG_1024   //!< 1024 samples
} ina260_averaging_mode_t;

/**
 * Conversion time
 */
typedef enum
{
    INA260_CT_140 = 0, //!< 140 us
    INA260_CT_204,     //!< 204 us
    INA260_CT_332,     //!< 332 us
    INA260_CT_588,     //!< 588 us
    INA260_CT_1100,    //!< 1.1 ms, default
    INA260_CT_2116,    //!< 2.116 ms
    INA260_CT_4156,    //!< 4.156 ms
    INA260_CT_8244,    //!< 8.244 ms
} ina260_conversion_time_t;

/**
 * Operating mode
 */
typedef enum
{
    INA260_MODE_POWER_DOWN     = 0, //!< Power-done
    INA260_MODE_TRIG_SHUNT     = 1, //!< Shunt current, triggered
    INA260_MODE_TRIG_BUS       = 2, //!< Bus voltage, triggered
    INA260_MODE_TRIG_SHUNT_BUS = 3, //!< Shunt current and bus voltage, triggered
    INA260_MODE_POWER_DOWN2    = 4, //!< Power-done
    INA260_MODE_CONT_SHUNT     = 5, //!< Shunt current, continuous
    INA260_MODE_CONT_BUS       = 6, //!< Bus voltage, continuous
    INA260_MODE_CONT_SHUNT_BUS = 7  //!< Shunt current and bus voltage, continuous (default)
} ina260_mode_t;

/**
 * Alert function mode
 */
typedef enum
{
    INA260_ALERT_DISABLED, //!< No alert function
    INA260_ALERT_OCL,      //!< Over current limit
    INA260_ALERT_UCL,      //!< Under current limit
    INA260_ALERT_BOL,      //!< Bus voltage over-voltage
    INA260_ALERT_BUL,      //!< Bus voltage under-voltage
    INA260_ALERT_POL,      //!< Power over-limit
} ina260_alert_mode_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev; //!< I2C device descriptor
    uint16_t config;   //!< Current config
    uint16_t mfr_id;   //!< Manufacturer ID
    uint16_t die_id;   //!< Die ID
} ina260_t;

/**
 * @brief Initialize device descriptor.
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t ina260_init_desc(ina260_t *dev, uint8_t addr, i2c_port_t port,
                           gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina260_free_desc(ina260_t *dev);

/**
 * @brief Initialize device.
 *
 * Reads sensor configuration.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina260_init(ina260_t *dev);

/**
 * @brief Reset sensor.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina260_reset(ina260_t *dev);

/**
 * @brief Configure sensor.
 *
 * @param dev Device descriptor
 * @param mode Operating mode
 * @param avg_mode Averaging mode
 * @param vbus_ct Bus voltage conversion time
 * @param ish_ct Shunt current conversion time
 * @return `ESP_OK` on success
 */
esp_err_t ina260_set_config(ina260_t *dev, ina260_mode_t mode, ina260_averaging_mode_t avg_mode,
                            ina260_conversion_time_t vbus_ct, ina260_conversion_time_t ish_ct);

/**
 * @brief Read sensor configuration.
 *
 * @param dev Device descriptor
 * @param[out] mode Operating mode
 * @param[out] avg_mode Averaging mode
 * @param[out] vbus_ct Bus voltage conversion time
 * @param[out] ish_ct Shunt current conversion time
 * @return `ESP_OK` on success
 */
esp_err_t ina260_get_config(ina260_t *dev, ina260_mode_t *mode, ina260_averaging_mode_t *avg_mode,
                            ina260_conversion_time_t *vbus_ct, ina260_conversion_time_t *ish_ct);

/**
 * @brief Setup ALERT pin.
 *
 * @param dev Device descriptor
 * @param mode Alert function mode
 * @param limit Alert limit value
 * @param cvrf If true also assert ALERT pin when device is ready to next conversion
 * @param active_high Set active ALERT pin level is high
 * @param latch Enable latch mode on ALERT pin (see ::ina260_get_status())
 * @return `ESP_OK` on success
 */
esp_err_t ina260_set_alert(ina260_t *dev, ina260_alert_mode_t mode, float limit,
                           bool cvrf, bool active_high, bool latch);

/**
 * @brief Trigger single conversion.
 *
 * Function will return an error if current operating
 * mode is not `INA260_MODE_TRIG_SHUNT`/`INA260_MODE_TRIG_BUS`/`INA260_MODE_TRIG_SHUNT_BUS`
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina260_trigger(ina260_t *dev);

/**
 * @brief Get device status.
 *
 * This function also clears ALERT state if latch mode is enabled for ALERT pin.
 *
 * @param dev Device descriptor
 * @param[out] ready If true, device is ready for the next conversion
 * @param[out] alert If true, there was alert
 * @param[out] overflow If true, power data have exceeded max 419.43 W
 * @return `ESP_OK` on success
 */
esp_err_t ina260_get_status(ina260_t *dev, bool *ready, bool *alert, bool *overflow);

/**
 * @brief Read current.
 *
 * This function works properly only after calibration.
 *
 * @param dev Device descriptor
 * @param[out] current Current, A
 * @return `ESP_OK` on success
 */
esp_err_t ina260_get_current(ina260_t *dev, float *current);

/**
 * @brief Read bus voltage.
 *
 * @param dev Device descriptor
 * @param[out] voltage Bus voltage, V
 * @return `ESP_OK` on success
 */
esp_err_t ina260_get_bus_voltage(ina260_t *dev, float *voltage);

/**
 * @brief Read power.
 *
 * This function works properly only after calibration.
 *
 * @param dev Device descriptor
 * @param[out] power Power, W
 * @return `ESP_OK` on success
 */
esp_err_t ina260_get_power(ina260_t *dev, float *power);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __INA260_H__ */

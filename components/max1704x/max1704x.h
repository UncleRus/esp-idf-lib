/*
 * Copyright (c) 2022 Joshua Butler, MD, MHI <josh.butler929@gmail.com>
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
 * @file max1704x.h
 * @defgroup max1704x max1704x
 * @{
 *
 * ESP-IDF driver for MAX17043/MAX17044/MAX17048/MAX17049 battery fuel gauge
 * 
 * Copyright (c) 2022 Joshua Butler, MD, MHI <josh.butler929@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __MAX1704X__H__
#define __MAX1704X__H__

#include <i2cdev.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX1704X_I2C_ADDR 0x36

/**
 * Device model
 */

typedef enum
{
    MAX17043_4 = 0,
    MAX17048_9,
} max1704x_model_t;

/**
 * Alert Status structure
 */
typedef struct
{
    bool reset_indicator;           //!< Reset indicator
    bool voltage_high;              //!< Voltage high alert
    bool voltage_low;               //!< Voltage low alert
    bool voltage_reset;             //!< Voltage reset alert
    bool soc_low;                   //!< SOC low alert, set when SOC cross empty_alert_thresh
    bool soc_change;                //!< SOC change alert, set when SOC change is at least 1%
    bool vreset_alert;              //!< Set to enable voltage reset alert under conditions specified in the valert register
} max1704x_status_t;

/**
 * MAX1704X configuration structure
 */
typedef struct 
{
    uint8_t rcomp;                  //!< RCOMP register value - default 0x97
    bool sleep_mode;                //!< Sleep mode - set to true to enter sleep mode
    bool soc_change_alert;          //!< SOC change alert - enable/disable SOC change alert
    bool alert_status;              //!< Alert status - read to check if alert has been triggered
    uint8_t empty_alert_thresh;     //!< Empty alert threshold - default 0x1C (4%, 32 - ATHD)
    uint8_t active_threshold;       //!< Exits hibernation when IOCV-CELLI above this threshold
    uint8_t hibernate_threshold;    //!< Enters Hibernation when CRATE falls below this threshold
} max1704x_config_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    max1704x_model_t model;
    max1704x_config_t config;
    max1704x_status_t status;
} max1704x_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_init_desc(max1704x_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_free_desc(max1704x_t *dev);

/**
 * @brief Quickstart battery fuel gauge
 * 
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_quickstart(max1704x_t *dev);

/**
 * @brief Get battery voltage
 * 
 * @param dev Device descriptor
 * @param voltage Battery voltage
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_get_voltage(max1704x_t *dev, float *voltage);

/**
 * @brief Get state of charge
 * 
 * @param dev Device descriptor
 * @param soc State of charge
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_get_soc(max1704x_t *dev, float *soc);

/**
 * @brief Get rate of battery charge or discharge
 * 
 * @param dev Device descriptor
 * @param crate Rate of charge or discharge
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_get_crate(max1704x_t *dev, float *crate);

/**
 * @brief Get the production version of the chip
 * 
 * @param dev Device descriptor
 * @param version Production version
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_get_version(max1704x_t *dev, uint16_t *version);

/**
 * @brief Get the configuration register
 * 
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_get_config(max1704x_t *dev);

/**
 * @brief Set the configuration register
 * 
 * @param dev Device descriptor
 * @param config Configuration register
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_set_config(max1704x_t *dev, max1704x_config_t *config);

/**
 * @brief Get the status register
 * 
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max1704x_get_status(max1704x_t *dev);

/**
 * @brief Set the status register
 * 
 * @param dev Device descriptor
 * @param status Status register
 * @return `ESP_OK` on success
 * 
 * @note Use this function to clear alert flags after servicing the alert
 */
esp_err_t max1704x_set_status(max1704x_t *dev, max1704x_status_t *status);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __MAX1704X__H__

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
 * @file max31865.h
 * @defgroup max31865 max31865
 * @{
 *
 * ESP-IDF driver for MAX31865, resistance converter for platinum RTDs
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MAX31865_H__
#define __MAX31865_H__

#include <stdint.h>
#include <stdbool.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX31865_MAX_CLOCK_SPEED_HZ (1000000) // 1 MHz

/**
 * Conversion mode
 */
typedef enum {
    MAX31865_MODE_SINGLE = 0, /**< Single consersion mode, default */
    MAX31865_MODE_AUTO        /**< Automatic conversion mode at 50/60Hz rate */
} max31865_mode_t;

/**
 * Notch frequencies for the noise rejection filter
 */
typedef enum {
    MAX31865_FILTER_60HZ = 0, /**< 60Hz */
    MAX31865_FILTER_50HZ      /**< 50Hz */
} max31865_filter_t;

/**
 * Connection type
 */
typedef enum {
    MAX31865_2WIRE = 0, /**< 2 wires */
    MAX31865_3WIRE,     /**< 3 wires */
    MAX31865_4WIRE      /**< 4 wires */
} max31865_connection_type_t;

/**
 * Device configuration
 */
typedef struct
{
    max31865_mode_t mode;
    max31865_connection_type_t connection;
    bool v_bias;
    max31865_filter_t filter;
} max31865_config_t;

/**
 * Temperature scale standard
 */
typedef enum {
    MAX31865_ITS90 = 0,    /**< ITS-90 */
    MAX31865_DIN43760,     /**< DIN43760 */
    MAX31865_US_INDUSTRIAL /**< US INDUSTRIAL */
} max31865_standard_t;

/**
 * Device descriptor
 */
typedef struct
{
    spi_device_interface_config_t spi_cfg;  /**< SPI device configuration */
    spi_device_handle_t spi_dev;            /**< SPI device handler */
    max31865_standard_t standard;           /**< Temperature scale standard */
    float r_ref;                            /**< Reference resistor value, Ohms */
    float rtd_nominal;                      /**< RTD nominal resistance at 0 deg. C, Ohms (PT100 - 100 Ohms, PT1000 - 1000 Ohms) */
} max31865_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev            Device descriptor
 * @param host           SPI host
 * @param clock_speed_hz SPI clock speed, Hz
 * @param cs_pin         CS GPIO number
 * @return `ESP_OK` on success
 */
esp_err_t max31865_init_desc(max31865_t *dev, spi_host_device_t host, uint32_t clock_speed_hz, gpio_num_t cs_pin);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max31865_free_desc(max31865_t *dev);

/**
 * @brief Configure device
 *
 * @param dev    Device descriptor
 * @param config Configuration
 * @return `ESP_OK` on success
 */
esp_err_t max31865_set_config(max31865_t *dev, const max31865_config_t *config);

/**
 * @brief Read device configuration
 *
 * @param dev         Device descriptor
 * @param[out] config Configuration
 * @return `ESP_OK` on success
 */
esp_err_t max31865_get_config(max31865_t *dev, max31865_config_t *config);

/**
 * @brief Trigger single-shot measurement
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max31865_start_measurement(max31865_t *dev);

/**
 * @brief Read raw RTD value
 *
 * @param dev        Device descritptor
 * @param[out] raw   Raw 15 bits RTD value
 * @param[out] fault true when fault is detected
 * @return `ESP_OK` on success
 */
esp_err_t max31865_read_raw(max31865_t *dev, uint16_t *raw, bool *fault);

/**
 * @brief Read RTD value and convert it temperature
 *
 * @param dev       Device descritptor
 * @param[out] temp Temperature, deg. Celsius
 * @return `ESP_OK` on success
 */
esp_err_t max31865_read_temperature(max31865_t *dev, float *temp);

/**
 * @brief Measure temperature
 *
 * Run full cycle of single-shot measurement:
 *  - trigger measurement
 *  - wait for 70ms
 *  - read and convert RTD value
 *
 * @param dev       Device descritptor
 * @param[out] temp Temperature, deg. Celsius
 * @return `ESP_OK` on success
 */
esp_err_t max31865_measure(max31865_t *dev, float *temp);

/**
 * @brief Run automatical fault detection cycle
 *
 * After calling this function, device must be reconfigured
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max31865_detect_fault_auto(max31865_t *dev);

/**
 * @brief Get bits of current fault status
 *
 * See datasheet for fault status interpretation
 *
 * @param dev               Device descriptor
 * @param[out] fault_status Fault status bits
 * @return `ESP_OK` on success
 */
esp_err_t max31865_get_fault_status(max31865_t *dev, uint8_t *fault_status);

/**
 * @brief Clear current fault status
 *
 * After calling this function, device must be reconfigured
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max31865_clear_fault_status(max31865_t *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MAX31865_H__ */

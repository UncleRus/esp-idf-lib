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
 * @file ccs811.h
 * @defgroup ccs811 ccs811
 * @{
 *
 * ESP-IDF driver for AMS CCS811 digital gas sensor connected to I2C
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>\n
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __CCS811_H__
#define __CCS811_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

//!< CCS811 I2C addresses
#define CCS811_I2C_ADDRESS_1      0x5a      //!< default
#define CCS811_I2C_ADDRESS_2      0x5b

#define CCS811_ERR_BASE 0xa000

//!< CCS811 driver error codes ORed with error codes for I2C the interface
#define CCS811_ERR_BOOT_MODE      (CCS811_ERR_BASE + 1) //!< firmware is in boot mode
#define CCS811_ERR_NO_APP         (CCS811_ERR_BASE + 2) //!< no application firmware loaded
#define CCS811_ERR_NO_NEW_DATA    (CCS811_ERR_BASE + 3) //!< no new data samples are ready
#define CCS811_ERR_NO_IAQ_DATA    (CCS811_ERR_BASE + 4) //!< no new data samples are ready
#define CCS811_ERR_HW_ID          (CCS811_ERR_BASE + 5) //!< wrong hardware ID
#define CCS811_ERR_INV_SENS       (CCS811_ERR_BASE + 6) //!< invalid sensor ID
#define CCS811_ERR_WR_REG_INV     (CCS811_ERR_BASE + 7) //!< invalid register addr on write
#define CCS811_ERR_RD_REG_INV     (CCS811_ERR_BASE + 8) //!< invalid register addr on read
#define CCS811_ERR_MM_INV         (CCS811_ERR_BASE + 9) //!< invalid measurement mode
#define CCS811_ERR_MAX_RESIST     (CCS811_ERR_BASE + 10) //!< max sensor resistance reached
#define CCS811_ERR_HEAT_FAULT     (CCS811_ERR_BASE + 11) //!< heater current not in range
#define CCS811_ERR_HEAT_SUPPLY    (CCS811_ERR_BASE + 12) //!< heater voltage not correct
#define CCS811_ERR_WRONG_MODE     (CCS811_ERR_BASE + 13) //!< wrong measurement mode
#define CCS811_ERR_RD_STAT_FAILED (CCS811_ERR_BASE + 14) //!< read status register failed
#define CCS811_ERR_RD_DATA_FAILED (CCS811_ERR_BASE + 15) //!< read sensor data failed
#define CCS811_ERR_APP_START_FAIL (CCS811_ERR_BASE + 16) //!< sensor app start failure
#define CCS811_ERR_WRONG_PARAMS   (CCS811_ERR_BASE + 17) //!< wrong parameters used

// ranges
#define CCS_ECO2_RANGE_MIN 400
#define CCS_ECO2_RANGE_MAX 8192
#define CCS_TVOC_RANGE_MIN 0
#define CCS_TVOC_RANGE_MAX 1187

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CCS811 operation modes
 */
typedef enum
{
    CCS811_MODE_IDLE = 0, //!< Idle, low current mode
    CCS811_MODE_1S = 1,   //!< Constant Power mode, IAQ values every 1 s
    CCS811_MODE_10S = 2,  //!< Pulse Heating mode, IAQ values every 10 s
    CCS811_MODE_60S = 3,  //!< Low Power Pulse Heating, IAQ values every 60 s
    CCS811_MODE_250MS = 4 //!< Constant Power mode, RAW data every 250 ms
} ccs811_mode_t;

/**
 * @brief CCS811 sensor device data structure
 */
typedef struct
{
    i2c_dev_t i2c_dev;  //!< I2C device handle
    ccs811_mode_t mode; //!< operation mode
} ccs811_dev_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Pointer to the sensor device data structure
 * @param addr Sensor address
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @returns ESP_OK on success
 */
esp_err_t ccs811_init_desc(ccs811_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Pointer to the sensor device data structure
 * @returns ESP_OK on success
 */
esp_err_t ccs811_free_desc(ccs811_dev_t *dev);

/**
 * @brief Initialize a CCS811 sensor
 *
 * The function initializes the CCS811 sensor and checks its availability.
 *
 * @param dev Pointer to the sensor device data structure
 *
 * @returns ESP_OK on success
 */
esp_err_t ccs811_init(ccs811_dev_t *dev);

/**
 * @brief Set the operation mode of the sensor
 *
 * The function sets the operating mode of the sensor. If the parameter
 * *mode* is either ::CCS811_MODE_1S, ::CCS811_MODE_10S, ::CCS811_MODE_60S
 * or ::CCS811_MODE_250MS, the sensor starts a periodic measurement with
 * the specified period. Function ::ccs811_get_results() can then be used at
 * the same rate to get the results.
 *
 * In ::CCS811_MODE_1S, ::CCS811_MODE_10S and ::CCS811_MODE_60S, raw sensor
 * data as well as IAQ values calculated by the  sensor values are available.
 * In ::CCS811_MODE_250MS, only raw data are available.
 *
 * In case, parameter mode is ::CCS811_MODE_IDLE, the sensor does not perform
 * any measurements.
 *
 * Please note: Mode timings are subject to typical 2% tolerance due
 * to accuracy of internal sensor clock.
 *
 * Please note: After setting the sensor mode, the sensor needs up to
 * 20 minutes, before accurate readings are generated.
 *
 * Please note: When a sensor operating mode is changed to a new mode with
 * a lower sample rate, e.g., from ::CCS811_MODE_60S to ::CCS811_MODE_1S, it
 * should be placed in *mode_idle* for at least 10 minutes before enabling
 * the new mode.
 *
 * @param dev Pointer to the sensor device data structure
 * @param mode Operation mode of the sensor
 *
 * @returns ESP_OK on success
 */
esp_err_t ccs811_set_mode(ccs811_dev_t *dev, ccs811_mode_t mode);

/**
 * @brief Get latest IAQ sensor values and/or RAW sensor data
 *
 * The function reads the IAQ sensor values (TVOC and eCO2) and/or the raw
 * sensor data. If some of the results are not needed, the corresponding
 * pointer parameters can be set to NULL.
 *
 * Please note: If the function is called and no new data are available,
 * e.g., due to the sensor mode time tolerance of 2%, the function still
 * returns successfully. In this case, the results of the last measurement
 * are returned and the error code CCS811_ERR_NO_NEW_DATA is set.
 *
 * Please note: In ::CCS811_MODE_250MS, only RAW data are available. In
 * that case, the function fails with error_code CCS811_ERR_NO_IAQ_DATA
 * if parameters *iaq_tvoc* and *iaq_eco2* are not NULL.
 *
 * @param dev pointer to the sensor device data structure
 * @param iaq_tvoc TVOC total volatile organic compound (0 - 1187 ppb)
 * @param iaq_eco2 eCO2 equivalent CO2 (400 - 8192 ppm)
 * @param raw_i current through the sensor used for measuring (0 - 63 uA)
 * @param raw_v voltage across the sensor measured (0 - 1023 = 1.65 V)
 *
 * @returns ESP_OK on success
 */
esp_err_t ccs811_get_results(ccs811_dev_t *dev, uint16_t *iaq_tvoc, uint16_t *iaq_eco2, uint8_t *raw_i, uint16_t *raw_v);

/**
 * @brief Get the resistance of connected NTC thermistor
 *
 * CCS811 supports an external interface for connecting a negative thermal
 * coefficient thermistor (R_NTC) to provide a cost effective and power
 * efficient means of calculating the local ambient temperature. The sensor
 * measures the voltage V_NTC across the R_NTC as well as the voltage V_REF
 * across a connected reference resistor (R_REF).
 * The function returns the current resistance of R_NTC using the equation
 *
 *          R_NTC = R_REF / V_REF * V_NTC
 *
 * Using the data sheet of the NTC, the ambient temperature can be calculated.
 *
 * @param dev pointer to the sensor device data structure
 * @param r_ref resistance of R_REF in Ohm
 * @param[out] res resistance of R_NTC in Ohm
 * @returns ESP_OK on success
 */
esp_err_t ccs811_get_ntc_resistance(ccs811_dev_t *dev, uint32_t r_ref, uint32_t *res);

/**
 * @brief Set environmental data
 *
 * If information about the environment are available from another sensor,
 * they can be used by CCS811 to compensate gas readings due to
 * temperature and humidity changes.
 *
 * @param dev pointer to the sensor device data structure
 * @param temperature measured temperature in degree Celsius
 * @param humidity measured relative humidity in percent
 * @returns ESP_OK on success
 */
esp_err_t ccs811_set_environmental_data(ccs811_dev_t *dev, float temperature, float humidity);

/**
 * @brief Enable or disable data ready interrupt signal *nINT*
 *
 * At the end of each measurement cycle (250ms, 1s, 10s, 60s), CCS811 can
 * optionally trigger an interrupt. The signal *nINT* is driven low as soon
 * as new sensor values are ready to read. It will stop being driven low
 * when sensor data are read with function *ccs811_get_results*.
 *
 * The interrupt is disabled by default.
 *
 * @param dev pointer to the sensor device data structure
 * @param enabled if true, the interrupt is enabled, or disabled otherwise
 * @returns ESP_OK on success
 */
esp_err_t ccs811_enable_interrupt(ccs811_dev_t *dev, bool enabled);

/**
 * @brief Set eCO2 threshold mode for data ready interrupts
 *
 * The user task can choose that the data ready interrupt is not generated
 * every time when new sensor values become ready but only if the eCO2 value
 * moves from the current range (LOW, MEDIUM, or HIGH) into another range by
 * more than a hysteresis value. Hysteresis is used to prevent multiple
 * interrupts close to a threshold.
 *
 *   - LOW     below parameter value *low*
 *   - MEDIUM  between parameter values *low* and *high*
 *   - HIGH    above parameter value *high* is range HIGH.
 *
 * If all parameters have valid values, the function sets the thresholds and
 * enables the data ready interrupt. Using 0 for all parameters disables the
 * interrupt.
 *
 * The interrupt is disabled by default.
 *
 * @param dev pointer to the sensor device data structure
 * @param low threshold LOW to MEDIUM  (>  400, default 1500)
 * @param high threshold MEDIUM to HIGH (< 8192, default 2500)
 * @param hysteresis hysteresis value (default 50)
 * @returns ESP_OK on success
 */
esp_err_t ccs811_set_eco2_thresholds(ccs811_dev_t *dev, uint16_t low, uint16_t high, uint8_t hysteresis);

/**
 * @brief Get the current baseline value from sensor
 *
 * The sensor supports automatic baseline correction over a minimum time of
 * 24 hours. Using this function, the current baseline value can be saved
 * before the sensor is powered down. This baseline can then be restored after
 * sensor is powered up again to continue the automatic baseline process.
 *
 * @param dev pointer to the sensor device data structure
 * @param[out] baseline current baseline value on success, or 0 on error
 * @returns ESP_OK on success
 */
esp_err_t ccs811_get_baseline(ccs811_dev_t *dev, uint16_t *baseline);

/**
 * @brief Write a previously stored baseline value to the sensor
 *
 * The sensor supports automatic baseline correction over a minimum time of
 * 24 hours. Using this function, a previously saved baseline value be
 * restored after the sensor is powered up to continue the automatic baseline
 * process.
 *
 * Please note: The baseline must be written after the conditioning period
 * of 20 min after power up.
 *
 * @param dev pointer to the sensor device data structure
 * @param baseline baseline to be set
 * @returns ESP_OK on success
 */
esp_err_t ccs811_set_baseline(ccs811_dev_t *dev, uint16_t baseline);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

/**@}*/

#endif /* __CCS811_H__ */


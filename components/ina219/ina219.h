/*
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
 * @file ina219.h
 * @defgroup ina219 ina219
 * @{
 *
 * ESP-IDF driver for INA219/INA220 Zerø-Drift, Bidirectional
 * Current/Power Monitor
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __INA219_H__
#define __INA219_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INA219_ADDR_GND_GND 0x40 //!< I2C address, A1 pin - GND, A0 pin - GND
#define INA219_ADDR_GND_VS  0x41 //!< I2C address, A1 pin - GND, A0 pin - VS+
#define INA219_ADDR_GND_SDA 0x42 //!< I2C address, A1 pin - GND, A0 pin - SDA
#define INA219_ADDR_GND_SCL 0x43 //!< I2C address, A1 pin - GND, A0 pin - SCL
#define INA219_ADDR_VS_GND  0x44 //!< I2C address, A1 pin - VS+, A0 pin - GND
#define INA219_ADDR_VS_VS   0x45 //!< I2C address, A1 pin - VS+, A0 pin - VS+
#define INA219_ADDR_VS_SDA  0x46 //!< I2C address, A1 pin - VS+, A0 pin - SDA
#define INA219_ADDR_VS_SCL  0x47 //!< I2C address, A1 pin - VS+, A0 pin - SCL
#define INA219_ADDR_SDA_GND 0x48 //!< I2C address, A1 pin - SDA, A0 pin - GND
#define INA219_ADDR_SDA_VS  0x49 //!< I2C address, A1 pin - SDA, A0 pin - VS+
#define INA219_ADDR_SDA_SDA 0x4a //!< I2C address, A1 pin - SDA, A0 pin - SDA
#define INA219_ADDR_SDA_SCL 0x4b //!< I2C address, A1 pin - SDA, A0 pin - SCL
#define INA219_ADDR_SCL_GND 0x4c //!< I2C address, A1 pin - SCL, A0 pin - GND
#define INA219_ADDR_SCL_VS  0x4d //!< I2C address, A1 pin - SCL, A0 pin - VS+
#define INA219_ADDR_SCL_SDA 0x4e //!< I2C address, A1 pin - SCL, A0 pin - SDA
#define INA219_ADDR_SCL_SCL 0x4f //!< I2C address, A1 pin - SCL, A0 pin - SCL

/**
 * Bus voltage range
 */
typedef enum {
    INA219_BUS_RANGE_16V = 0, //!< 16V FSR
    INA219_BUS_RANGE_32V      //!< 32V FSR (default)
} ina219_bus_voltage_range_t;

/**
 * PGA gain for shunt voltage
 */
typedef enum {
    INA219_GAIN_1 = 0, //!< Gain: 1, Range: +-40 mV
    INA219_GAIN_0_5,   //!< Gain: 1/2, Range: +-80 mV
    INA219_GAIN_0_25,  //!< Gain: 1/4, Range: +-160 mV
    INA219_GAIN_0_125  //!< Gain: 1/8, Range: +-320 mV (default)
} ina219_gain_t;

/**
 * ADC resolution/averaging
 */
typedef enum {
    INA219_RES_9BIT_1S    = 0,  //!< 9 bit, 1 sample, conversion time 84 us
    INA219_RES_10BIT_1S   = 1,  //!< 10 bit, 1 sample, conversion time 148 us
    INA219_RES_11BIT_1S   = 2,  //!< 11 bit, 1 sample, conversion time 276 us
    INA219_RES_12BIT_1S   = 3,  //!< 12 bit, 1 sample, conversion time 532 us (default)
    INA219_RES_12BIT_2S   = 9,  //!< 12 bit, 2 samples, conversion time 1.06 ms
    INA219_RES_12BIT_4S   = 10, //!< 12 bit, 4 samples, conversion time 2.13 ms
    INA219_RES_12BIT_8S   = 11, //!< 12 bit, 8 samples, conversion time 4.26 ms
    INA219_RES_12BIT_16S  = 12, //!< 12 bit, 16 samples, conversion time 8.51 ms
    INA219_RES_12BIT_32S  = 13, //!< 12 bit, 32 samples, conversion time 17.02 ms
    INA219_RES_12BIT_64S  = 14, //!< 12 bit, 64 samples, conversion time 34.05 ms
    INA219_RES_12BIT_128S = 15, //!< 12 bit, 128 samples, conversion time 68.1 ms
} ina219_resolution_t;

/**
 * Operating mode
 */
typedef enum {
    INA219_MODE_POWER_DOWN = 0, //!< Power-done
    INA219_MODE_TRIG_SHUNT,     //!< Shunt voltage, triggered
    INA219_MODE_TRIG_BUS,       //!< Bus voltage, triggered
    INA219_MODE_TRIG_SHUNT_BUS, //!< Shunt and bus, triggered
    INA219_MODE_DISABLED,       //!< ADC off (disabled)
    INA219_MODE_CONT_SHUNT,     //!< Shunt voltage, continuous
    INA219_MODE_CONT_BUS,       //!< Bus voltage, continuous
    INA219_MODE_CONT_SHUNT_BUS  //!< Shunt and bus, continuous (default)
} ina219_mode_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;

    uint16_t config;
    float i_lsb, p_lsb;
} ina219_t;

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
esp_err_t ina219_init_desc(ina219_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_free_desc(ina219_t *dev);

/**
 * @brief Init device
 *
 * Read current device configuration into `dev->config`
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_init(ina219_t *dev);

/**
 * @brief Reset device
 *
 * Same as power-on reset. Resets all registers to default values.
 * You still need to calibrate device to read current, otherwise
 * only shunt voltage readings will be valid.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_reset(ina219_t *dev);

/**
 * @brief Set device configuration
 *
 * @param dev Device descriptor
 * @param u_range Bus voltage range
 * @param gain Shunt voltage gain
 * @param u_res Bus voltage resolution and averaging
 * @param i_res Shunt voltage resolution and averaging
 * @param mode Device operational mode
 * @return `ESP_OK` on success
 */
esp_err_t ina219_configure(ina219_t *dev, ina219_bus_voltage_range_t u_range,
        ina219_gain_t gain, ina219_resolution_t u_res,
        ina219_resolution_t i_res, ina219_mode_t mode);

/**
 * @brief Get bus voltage range
 *
 * @param dev Device descriptor
 * @param[out] range Bus voltage range
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage_range(ina219_t *dev, ina219_bus_voltage_range_t *range);

/**
 * @brief Get shunt voltage gain
 *
 * @param dev Device descriptor
 * @param[out] gain Shunt voltage gain
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_gain(ina219_t *dev, ina219_gain_t *gain);

/**
 * @brief Get bus voltage resolution and averaging
 *
 * @param dev Device descriptor
 * @param[out] res Bus voltage resolution and averaging
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage_resolution(ina219_t *dev, ina219_resolution_t *res);

/**
 * @brief Get shunt voltage resolution and averaging
 *
 * @param dev Device descriptor
 * @param[out] res Shunt voltage resolution and averaging
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_shunt_voltage_resolution(ina219_t *dev, ina219_resolution_t *res);

/**
 * @brief Get operating mode
 *
 * @param dev Device descriptor
 * @param[out] mode Operating mode
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_mode(ina219_t *dev, ina219_mode_t *mode);

/**
 * @brief Perform calibration
 *
 * Current readings will be valid only after calibration
 *
 * @param dev Device descriptor
 * @param i_expected_max Maximum expected current, A
 * @param r_shunt Shunt resistance, Ohm
 * @return `ESP_OK` on success
 */
esp_err_t ina219_calibrate(ina219_t *dev, float i_expected_max, float r_shunt);

/**
 * @brief Trigger single conversion
 *
 * Function will return an error if current operating
 * mode is not `INA219_MODE_TRIG_SHUNT`/`INA219_MODE_TRIG_BUS`/`INA219_MODE_TRIG_SHUNT_BUS`
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_trigger(ina219_t *dev);

/**
 * @brief Read bus voltage
 *
 * @param dev Device descriptor
 * @param[out] voltage Bus voltage, V
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage(ina219_t *dev, float *voltage);

/**
 * @brief Read shunt voltage
 *
 * @param dev Device descriptor
 * @param[out] voltage Shunt voltage, V
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_shunt_voltage(ina219_t *dev, float *voltage);

/**
 * @brief Read current
 *
 * This function works properly only after calibration.
 *
 * @param dev Device descriptor
 * @param[out] current Current, A
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_current(ina219_t *dev, float *current);

/**
 * @brief Read power
 *
 * This function works properly only after calibration.
 *
 * @param dev Device descriptor
 * @param[out] power Power, W
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_power(ina219_t *dev, float *power);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __INA219_H__ */

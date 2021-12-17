/*
 * Copyright (c) 2021 saasaa <mail@saasaa.xyz>
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
 * @file hts221.h
 * @brief HTS221 driver
 * @author saasaa
 * @defgroup hts221 hts221
 * @{
 *
 * ESP-IDF driver for HTS221 temperature and humidity sensor
 *
 * Datasheet: http://www.st.com/resource/en/datasheet/hts221.pdf
 * Technical note on interpreting humidity and temperature readings in the
 * HTS221 digital humidity sensor:
 * https://www.st.com/resource/en/technical_note/tn1218-interpreting-humidity-and-temperature-readings-in-the-hts221-digital-humidity-sensor-stmicroelectronics.pdf
 * Copyright (c) 2021 saasaa <mail@saasaa.xyz>
 *
 * ISC Licensed as described in the file LICENSE
 *
 */

#ifndef __HTS221_H__
#define __HTS221_H__

#include <esp_err.h>
#include <i2cdev.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HTS221_I2C_ADDRESS 0x5F

#define HTS221_REG_WHOAMI 0x0F  // R   Device identification register
#define HTS221_REG_AV_CONF 0x10 // R/W Humidity and temperature resolution mode
#define HTS221_REG_CTRL_REG1 0x20 // R/W Control register 1
#define HTS221_REG_CTRL_REG2 0x21 // R/W Control register 2
#define HTS221_REG_CTRL_REG3                                                   \
    0x22 // R/W Control register 3 for data ready output signal
#define HTS221_REG_STATUS_REG 0x27 // R   Status register
#define HTS221_REG_HUMIDITY_OUT_L                                              \
    0x28 // R   Relative humidity output register (LSB)
#define HTS221_REG_HUMIDITY_OUT_H                                              \
    0x29 // R   Relative humidity output register (MSB)
#define HTS221_REG_TEMP_OUT_L 0x2A  // R   Temperature output register (LSB)
#define HTS221_REG_TEMP_OUT_H 0x2B  // R   Temperature output register (MSB)
#define HTS221_REG_CALIB_START 0x30 // R/W Calibration start register

#define HTS221_AV_CONF_DEFAULT                                                 \
    0x1B // DEFAULT AV_CONF status register value according to datasheet

#define HTS221_CTRL_REG1                                                       \
    0x85 // Set CTRL_REG1 to power on mode, enable block data update, set output
         // data rate to 1Hz
#define HTS221_CTRL_REG1_POWERON                                               \
    0x80 // CTRL_REG1 status register value to power on HTS221
#define HTS221_CTRL_REG1_POWERDOWN                                             \
    0x00 // CTRL_REG1 status register value to power on HTS221

#define HTS221_CTRL_REG2                                                       \
    0x00 // Default CTRL_REG2 status register
         // value according to datasheet
#define HTS221_CTRL_REG2_HEATER_ON                                             \
    0x02 // CTRL_REG2 status register value to enable heater
#define HTS221_CTRL_REG2_HEATER_OFF                                            \
    0x00 // CTRL_REG2 status register value to
         // disable heater

#define HTS221_CTRL_REG3                                                       \
    0x00 // Default CTRL_REG3 status register value according to datasheet

/**
 * @brief Struct for storing calibration parameters.
 *
 * Unique for every HTS221 Sensor.
 */
typedef struct
{
    uint8_t H0_rH;
    uint8_t H1_rH;
    uint16_t T0_degC;
    uint16_t T1_degC;
    int16_t H0_T0_OUT;
    int16_t H1_T0_OUT;
    int16_t T0_OUT;
    int16_t T1_OUT;
} calibration_param_t;

/**
 * @brief Initialize the HTS221 Device descriptor
 *
 * @param[out] dev Device descriptor
 * @param[in] addr I2C address
 * @param[in] port I2C port number
 * @param[in] sda_gpio GPIO pin number for SDA
 * @param[in] scl_gpio GPIO pin number for SCL
 * @return `ESP_OK` on success
 */
esp_err_t hts221_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port,
                           gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param[in] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hts221_free_desc(i2c_dev_t *dev);

/**
 * @brief Read the HTS221 WHO_AM_I register (0xBC)
 *
 * @param[in] dev Device descriptor
 * @param[out] who_am_i Device identifier, should be (OxBC)
 * @return `ESP_OK` on success
 */
esp_err_t hts221_who_am_i(i2c_dev_t *dev, uint8_t *who_am_i);

/**
 * @brief Setup device parameters
 *
 * Set the AV_CONF register to the default resolution mode (0x1B) according to
 * the datasheet.
 * Set CTRL_REG1 to power on mode, enable block data update, set output
 * data rate to 1Hz.
 * Set CTRL_REG2 to default value (0x00), BOOT bit normal, heater off, one-shot
 * set to 0.
 *
 * @param[in] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hts221_setup(i2c_dev_t *dev);

/**
 * @brief Read the HTS221 humidity and temperature resolution mode register
 *
 * @param[in] dev Device descriptor
 * @param[out] av_conf humidity and temperature resolution mode register value,
 * default (0x1B)
 * @return `ESP_OK` on success
 */
esp_err_t hts221_read_av_conf(i2c_dev_t *dev, uint8_t *av_conf);

/**
 * @brief Read the HTS221 calibration parameters from the respective registers
 * and save them in the static calibration_paramters struct.
 *
 * @param[in] dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hts221_read_calibration_coeff(i2c_dev_t *dev);

/**
 * @brief Get temperature and relative humidity
 *
 * @param dev Device descriptor
 * @param[out] temperature Temperature, degrees Celsius
 * @param[out] humidity    Relative humidity, percents
 * @return `ESP_OK` on success
 */
esp_err_t hts221_read_data(i2c_dev_t *dev, float *temperature, float *humidity);

/**
 * @brief Write the HTS221 humidity and temperature resolution mode register
 *
 * @param[in] dev Device descriptor
 * @param[in] av_conf Status register value, default (0x1B)
 * @return `ESP_OK` on success
 */
esp_err_t hts221_set_av_conf(i2c_dev_t *dev, uint8_t *av_conf);

/**
 * @brief Print the HTS221 calibration parameters via ESP_LOGI to serial output
 *
 * @return `ESP_OK` on success
 */
esp_err_t hts221_print_calibration_coeff();

/**
 * @brief Read the HTS221 status register values
 *
 * @param[in] dev Device descriptor
 * @param[out] status_reg Status register value, default (0x00)
 * @return `ESP_OK` on success
 */
esp_err_t hts221_read_status_register(i2c_dev_t *dev, uint8_t *status_reg);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __HTS221_H__

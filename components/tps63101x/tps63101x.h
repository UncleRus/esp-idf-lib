/*
 * Copyright (c) 2024 Manuel Markwort <https://github.com/mmarkwort>
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
 * @file tps63101x.h
 * @defgroup tps63101x tps63101x
 * @{
 *
 * ESP-IDF driver for Texas Instruments TPS631012 and TPS631013 1.6-V to 5.5-V Input Voltage 1.5-A Buck-boost Converter with I2C
 *
 * Copyright (c) 2024 Manuel Markwort <https://github.com/mmarkwort>\n
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __TPS63101X_H__
#define __TPS63101X_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TPS63101X_I2C_ADDR 0x2A //!< I2C address

#define TPS63101X_CONTROL_1_DEFAULT 0x8 //!< Default value of Control 1 register
#define TPS63101X_VOUT_DEFAULT 0x5C //!< Default value of VOUT register
#define TPS63101X_CONTROL_2_DEFAULT 0x45 //!< Default value of Control 2 register

/**
 * Control 1 Register/Address: 02h (Default: 0x8)
 */
typedef struct
{
    union {
        struct {
            uint8_t converter_en : 1;   //!< Enable Converter ('AND'ed with EN-pin): 0 : DISABLE, 1 : ENABLE / Default: 0 / RW
            uint8_t nil2 : 1;           //!< Not used. / Default: 0 / R
            uint8_t en_scp : 1;         //!< Enable short circuit hiccup protection: 0 : DISABLE, 1 : ENABLE / Default: 0 / RW
            uint8_t en_fast_dvs : 1;    //!< Sets DVS to fast mode: 0 : DISABLE, 1 : ENABLE / Default: 1 / RW
            uint8_t nil : 4;            //!< Not used. During write operations data for these bits are ignored. During read operations 0 is returned. / Default: 0 / R
        } data_fields;
        struct {
            uint8_t reg;                //!< Register data
        } register_data;
    };
    
} tps63101x_control_1_t;

/**
 * Control 1 Register/Address: 03h (Default: 0x5C)
 */
typedef struct
{
    union {
        struct {
            uint8_t vout;               //!< These bits set the output voltage: Output voltage = 1.000 + (VOUT[7 :0] × 0.025) V when 0x00<=VOUT[7 :0]<=0xB4, Output voltage = 5.5 V when 0xB5<=VOUT[7 :0]<=0xFF / Default: 5C / RW
        } data_fields;
        struct {
            uint8_t reg;                //!< Register data
        } register_data;
    };
    
} tps63101x_vout_t;

/**
 * Control 2 Register/Address: 05h (Default: 0x45)
 */
typedef struct
{
    union {
        struct {
            uint8_t td_ramp : 3;        //!< Defines the ramp time for the Vo soft start ramp: 000: 0.256ms, 001: 0.512ms, 010: 1.024ms, 011: 1.920ms, 100: 3.584ms, 101: 7.552ms, 110: 9.600ms, 111: 24.320ms / Default: 101 / RW
            uint8_t cl_ramp_min : 1;    //!< Define the minimum current limit during the soft start ramp: 0 : Low (500mA), 1 : High (2x Low) / Default: 0 / RW
            uint8_t en_disch_vout : 2;  //!< Enable of BUBO Vout Discharge: 00 : DISABLE, 01 : SLOW (34mA), 10 : MEDIUM (67mA), 11 : FAST (100mA) / Default: 0 / RW
            uint8_t fast_ramp_en : 1;   //!< Device can start-up faster than VOUT ramp: 0 : DISABLE, 1 : ENABLE / Default: 1 / RW
            uint8_t fpwm : 1;           //!< Force PWM operation: 0 : DISABLE, 1 : ENABLE / Default: 0 / RW
        } data_fields;
        struct {
            uint8_t reg;                //!< Register data
        } register_data;
    };
    
} tps63101x_control_2_t;


/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param port      I2C port
 * @param sda_gpio  SDA GPIO pin
 * @param scl_gpio  SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_free_desc(i2c_dev_t *dev);

/**
 * @brief Reads control 1 register
 * @param dev Device descriptor
 * @param control_1 Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_get_control_1(i2c_dev_t *dev, tps63101x_control_1_t* control_1);

/**
 * @brief Writes control 1 register
 * @param dev Device descriptor
 * @param control_1 Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_set_control_1(i2c_dev_t *dev, tps63101x_control_1_t* control_1);

/**
 * @brief Reads vout register
 * @param dev Device descriptor
 * @param vout Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_get_vout(i2c_dev_t *dev, tps63101x_vout_t* vout);

/**
 * @brief Writes vout register
 * @param dev Device descriptor
 * @param vout Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_set_vout(i2c_dev_t *dev, tps63101x_vout_t* vout);

/**
 * @brief Reads control 2 register
 * @param dev Device descriptor
 * @param control_2 Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_get_control_2(i2c_dev_t *dev, tps63101x_control_2_t* control_2);

/**
 * @brief Writes control 2 register
 * @param dev Device descriptor
 * @param control_2 Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_set_control_2(i2c_dev_t *dev, tps63101x_control_2_t* control_2);

/**
 * @brief Resets all registers to default vaules
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tps63101x_reset(i2c_dev_t *dev);

/**
 * @brief Calculates the register value for the given value (min. voltage = 1.0, max. volatge = 5.5)
 * @param voltage Device descriptor
 * @return returns the register value of the given voltage on success `0xFF` on failure
 */
uint8_t tps63101x_to_register_voltage(float voltage);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif
/*
 * Copyright (c) 2023 Manuel Markwort <https://github.com/mmarkwort>
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
 * @file mp2660.h
 * @defgroup mp2660 mp2660
 * @{
 *
 * ESP-IDF driver for Monolithic Systems 5V USB, 500mA, I2C-Controlled Linear Charger with Power Path Management for Single-Cell Li-Ion Battery
 *
 * Copyright (c) 2023 Manuel Markwort <https://github.com/mmarkwort>\n
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __MP2660_H__
#define __MP2660_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MP2660_I2C_ADDR 0x09 //!< I2C address

#define MP2660_SOURCE_CONTROL_REG_DEFAULT 0x4F //!< Default value of source control register after power on
#define MP2660_POWER_ON_CONFIG_REG_DEFAULT 0x4 //!< Default value of power on configuration register after power on
#define MP2660_CURENT_CONTROL_REG_DEFAULT 0xE //!< Default value of current control register after power on
#define MP2660_PRECHARGE_TERMINATION_CURRENT_REG_DEFAULT 0x4A //!< Default value of pre charge termination current register after power on
#define MP2660_CHARGE_VOLTAGE_CONTROL_REG_DEFAULT 0xA3 //!< Default value of voltage control register after power on
#define MP2660_CHARGE_TERMINATION_TIMER_CONTROL_REG_DEFAULT 0x4A //!< Default value of termination timer control register after power on
#define MP2660_MISC_OPERATION_CONTROL_REG_DEFAULT 0xB //!< Default value of misc operation control register after power on
#define MP2660_SYSTEM_STATUS_REG_DEFAULT 0x0 //!< Default value of system status register after power on
#define MP2660_FAULT_REG_DEFAULT 0x0 //!< Default value of fault register after power on

/**
 * Input Source Control Register/Address: 00h (Default: 0100 1111)
 */
typedef struct
{
    union {
        struct {
            uint8_t i_in_lim_0 : 1;     //!< 000: 85mA, 001: 130mA, 010: 175mA, 011: 220mA, 100: 265mA, 101: 310mA, 110: 355mA, 111: 455mA, Default: 455mA (111)
            uint8_t i_in_lim_1 : 1;     //!< s.a.
            uint8_t i_in_lim_2 : 1;     //!< s.a.
            uint8_t v_in_min_0 : 1;     //!< 80mV / RW / VIn Min Offset: 3.88V, Range 3.88V - 5.08V, Default: 4.60V (1001)
            uint8_t v_in_min_1 : 1;     //!< 160mV / RW
            uint8_t v_in_min_2 : 1;     //!< 320mV / RW
            uint8_t v_in_min_3 : 1;     //!< 640mV / RW
            uint8_t en_hiz : 1;         //!< 0: Disable 1: Enable / RW / This bit only controls the on and off of the LDO FET.
        } data_fields;
        struct {
            uint8_t reg;                //!< Register data
        } register_data;
    };
    
} mp2660_input_source_t;

/**
 * Power-On Configuration Register / Address: 01h (Default: 0000 0100)
 */
typedef struct
{
    union {
        struct {
            uint8_t v_batt_uvlo_0 : 1;      //!< 0.1V / RW / Offset: 2.4V, Range: 2.4V - 3.1V, Default: 2.8V (100)
            uint8_t v_batt_uvlo_1 : 1;      //!< 0.2V / RW
            uint8_t v_batt_uvlo_2 : 1;      //!< 0.4V / RW
            uint8_t ceb : 1;                //!< 0: Charge enable 1: Charge disable / RW / Default: Charge enable (0)
            uint8_t reserved_1 : 1;         //!< NA
            uint8_t reserved_2 : 1;         //!< NA
            uint8_t i2c_watchdog_timer : 1; //!< 0: Normal 1: Reset / RW / Default: Normal (0)
            uint8_t reg_reset : 1;          //!< 0: Keep current setting 1: Reset / RW / Default: Keep current register setting (0)
        } data_fields;
        struct {
            uint8_t reg;                    //!< Register data
        } register_data;
    };
    
} mp2660_power_on_conf_t;

/**
 * Charge Current Control Register/ Address: 02h (Default: 0000 1110)
 */
typedef struct
{
    union {
        struct {
            uint8_t icc_4 : 1;      //!< 272mA / RW / Offset: 8mA, Range: 8mA - 535mA, Default: 246mA (01110)
            uint8_t icc_3 : 1;      //!< 136mA / RW
            uint8_t icc_2 : 1;      //!< 68mA / RW
            uint8_t icc_1 : 1;      //!< 34mA / RW
            uint8_t icc_0 : 1;      //!< 17mA / RW
            uint8_t reserved_3 : 1; //!< NA
            uint8_t reserved_2 : 1; //!< NA
            uint8_t reserved_1 : 1; //!< NA
        } data_fields;
        struct {
            uint8_t reg;            //!< Register data
        } register_data;
    };
    
} mp2660_charge_current_ctrl_t;

/**
 * Pre-Charge/ Termination Current/ Address: 03h (Default: 0100 1010)
 */
typedef struct
{
    union {
        struct {
            uint8_t reserved_1 : 1; //!< NA
            uint8_t i_dschg_3 : 1;  //!< 800mA / RW / Offset: 200mA, Range: 200mA - 1.6A, Default: 1.0A (1001)
            uint8_t i_dschg_2 : 1;  //!< 400mA / RW
            uint8_t i_dschg_1 : 1;  //!< 200mA / RW
            uint8_t i_dschg_0 : 1;  //!< 100mA / RW
            uint8_t reserved_2 : 1; //!< NA
            uint8_t i_pre_1 : 1;    //!< 14mA / RW / Offset: 6mA, Range: 6mA - 27mA, Default: 20mA (10)
            uint8_t i_pre_0 : 1;    //!< 7mA / RW
        } data_fields;
        struct {
            uint8_t reg;            //!< Register data
        } register_data;
    };
    
} mp2660_pre_charge_term_current_t;

/**
 * Charge Voltage Control Register/ Address: 04h (Default: 1010 0011)
 */
typedef struct
{
    union {
        struct {
            uint8_t v_bat_reg_5 : 1;    //!< 480mV / RW / Offset: 3.60V, Range: 3.60V - 4.545V, Default: 4.2V (101000)
            uint8_t v_bat_reg_4 : 1;    //!< 240mV / RW
            uint8_t v_bat_reg_3 : 1;    //!< 120mV / RW
            uint8_t v_bat_reg_2 : 1;    //!< 60mV / RW
            uint8_t v_bat_reg_1 : 1;    //!< 30mV / RW
            uint8_t v_bat_reg_0 : 1;    //!< 15mV / RW
            uint8_t v_batt_pre : 1;     //!< 0: 2.8V 1: 3.0V / RW / Default: 3.0V (1)
            uint8_t v_batt_rech : 1;    //!< 0: 150mV 1: 300mV / RW / Default: 300mV (1)
        } data_fields;
        struct {
            uint8_t reg;                //!< Register data
        } register_data;
    };
    
} mp2660_charge_voltage_ctrl_t;

/**
 * Charge Termination/Timer Control Register / Address: 05h (Default: 0100 1010)
 */
typedef struct
{
    union {
        struct {
            uint8_t reserved : 1;       //!< NA
            uint8_t en_term : 1;        //!< 0: Disable 1: Enable / RW / Default: Enable (1) 
            uint8_t watchdog_1 : 1;     //!< 00: Disable timer 01: 40s 10: 80s 11: 160s / RW / Default: Disable timer (00)
            uint8_t watchdog_0 : 1;     //!< s.a. 
            uint8_t en_timer : 1;       //!< 0: Disable 1: Enable / RW / Default: Enable timer (1) 
            uint8_t chg_timer_1 : 1;    //!< 00: 3hrs 01: 5hrs 10: 8hrs 11: 12hrs / RW / Default: 5hrs (01)
            uint8_t chg_timer_0 : 1;    //!< s.a. 
            uint8_t term_tmr : 1;       //!< 0: Disable 1: Enable / RW / Default: Disable (0) 
        } data_fields;
        struct {
            uint8_t reg;                //!< Register data
        } register_data;
    };
    
} mp2660_charge_term_timer_ctrl_t;

/**
 * Miscellaneous Operation Control Register/ Address: 06h (Default: 0000 1011)
 */
typedef struct
{
    union {
        struct {
            uint8_t reserved_1 : 1; //!< NA
            uint8_t tmr2x_en : 1;   //!< 0: Disable 2X extended safety timer during PPM 1: Enable 2X extended safety timer during PPM / RW / Default: Disable (0)
            uint8_t fet_dis : 1;    //!< 0: Enable 1: Turn off / RW / Default: Enable (0)
            uint8_t reserved_2 : 1; //!< NA
            uint8_t en_ntc : 1;     //!< 0: Disable 1: Enable / RW / Default: Enable (1)
            uint8_t reserved_3 : 1; //!< NA
            uint8_t tj_reg_0 : 1;   //!< 00: 60°C 01: 80°C 10: 100°C 11: 120°C / RW / 120°C (11)
            uint8_t tj_reg_1 : 1;   //!< s.a.
        } data_fields;
        struct {
            uint8_t reg;            //!< Register data
        } register_data;
    };
    
} mp2660_misc_op_ctrl_t;

/**
 * System Status Register/ Address: 07h (Default: 0000 0000)
 */
typedef struct
{
    union {
        struct {
            uint8_t reserved_1 : 1; //!< NA
            uint8_t rev_1 : 1;      //!< Revision number / R / Default: (00)
            uint8_t rev_0 : 1;      //!< s.a. 
            uint8_t chg_stat_1 : 1; //!< 00: Not charging 01: Pre-charge 10: Charge 11: Charge done / R / Default: Not charging (00)
            uint8_t chg_stat_0 : 1; //!< s.a. 
            uint8_t ppm_stat : 1;   //!< 0: No PPM 1: In PPM / R / Default: No PPM (0) (no power-path management happens)
            uint8_t pg_stat : 1;    //!< 0: Power fail 1: Power good / R / Default: Power fail (0)
            uint8_t therm_stat : 1; //!< 0: No thermal regulation 1: In thermal regulation / R / No thermal regulation (0)
        } data_fields;
        struct {
            uint8_t reg;            //!< Register data
        } register_data;
    };
    
} mp2660_sys_status_t;

/**
 * Fault Register/ Address: 08h (Default: 0000 0000)
 */
typedef struct
{
    union {
        struct {
            uint8_t reserved_1 : 1;     //!< NA
            uint8_t watchdog_fault : 1; //!< 0: Normal 1: Watchdog timer expiration / R / Default: Normal (0)
            uint8_t vin_fault : 1;      //!< 0: Normal 1: Input fault (OVP or bad source) / R / Default: Normal (0)
            uint8_t them_sd : 1;        //!< 0: Normal 1: Thermal shutdown / R / Default: Normal (0)
            uint8_t bat_fault : 1;      //!< 0: Normal 1: Battery OVP / R / Default: Normal (0)
            uint8_t stmr_fault : 1;     //!< 0: Normal 1: Safety timer expiration / R / Default: Normal (0)
            uint8_t reserved_2 : 1;     //!< NA
            uint8_t reserved_3 : 1;     //!< NA
        } data_fields;
        struct {
            uint8_t reg;                //!< Register data
        } register_data;
    };
    
} mp2660_fault_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param port      I2C port
 * @param sda_gpio  SDA GPIO pin
 * @param scl_gpio  SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_free_desc(i2c_dev_t *dev);

/**
 * @brief Reads input source register
 * @param dev Device descriptor
 * @param input_source Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_input_source(i2c_dev_t *dev, mp2660_input_source_t* input_source);

/**
 * @brief Writes input source register
 * @param dev Device descriptor
 * @param input_source Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_set_input_source(i2c_dev_t *dev, mp2660_input_source_t* input_source);

/**
 * @brief Reads power configuration register
 * @param dev Device descriptor
 * @param pwr_config Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_pwr_on_conf(i2c_dev_t *dev, mp2660_power_on_conf_t* pwr_config);

/**
 * @brief Writes power configuration register
 * @param dev Device descriptor
 * @param pwr_config Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_set_pwr_on_conf(i2c_dev_t *dev, mp2660_power_on_conf_t* pwr_config);

/**
 * @brief Reads charge current control register
 * @param dev Device descriptor
 * @param chrg_current_ctrl Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_chrg_current_ctrl(i2c_dev_t *dev, mp2660_charge_current_ctrl_t* chrg_current_ctrl);

/**
 * @brief Writes charge current control register
 * @param dev Device descriptor
 * @param chrg_current_ctrl Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_set_chrg_current_ctrl(i2c_dev_t *dev, mp2660_charge_current_ctrl_t* chrg_current_ctrl);

/**
 * @brief Reads pre-charge / termination current register
 * @param dev Device descriptor
 * @param pre_chrg_term_current Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_pre_chrg_term_current(i2c_dev_t *dev, mp2660_pre_charge_term_current_t* pre_chrg_term_current);

/**
 * @brief Writes pre-charge / termination current register
 * @param dev Device descriptor
 * @param pre_chrg_term_current Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_set_pre_chrg_term_current(i2c_dev_t *dev, mp2660_pre_charge_term_current_t* pre_chrg_term_current);

/**
 * @brief Reads charge voltage control register
 * @param dev Device descriptor
 * @param chrg_voltage_ctrl Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_chrg_voltage_control(i2c_dev_t *dev, mp2660_charge_voltage_ctrl_t* chrg_voltage_ctrl);

/**
 * @brief Writes charge voltage control register
 * @param dev Device descriptor
 * @param chrg_voltage_ctrl Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_set_chrg_voltage_control(i2c_dev_t *dev, mp2660_charge_voltage_ctrl_t* chrg_voltage_ctrl);

/**
 * @brief Reads charge termination / timer control register
 * @param dev Device descriptor
 * @param charge_term_timer_ctrl Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_chrg_term_timer_control(i2c_dev_t *dev, mp2660_charge_term_timer_ctrl_t* charge_term_timer_ctrl);

/**
 * @brief Writes charge termination / timer control register
 * @param dev Device descriptor
 * @param charge_term_timer_ctrl Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_set_chrg_term_timer_control(i2c_dev_t *dev, mp2660_charge_term_timer_ctrl_t* charge_term_timer_ctrl);

/**
 * @brief Reads miscellaneous operation control register
 * @param dev Device descriptor
 * @param misc_op_ctrl Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_misc_op_control(i2c_dev_t *dev, mp2660_misc_op_ctrl_t* misc_op_ctrl);

/**
 * @brief Writes miscellaneous operation control register
 * @param dev Device descriptor
 * @param misc_op_ctrl Source buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_set_misc_op_control(i2c_dev_t *dev, mp2660_misc_op_ctrl_t* misc_op_ctrl);

/**
 * @brief Reads system status register
 * @param dev Device descriptor
 * @param sys_status Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_sys_status(i2c_dev_t *dev, mp2660_sys_status_t* sys_status);

/**
 * @brief Reads fault register
 * @param dev Device descriptor
 * @param fault Target buffer
 * @return `ESP_OK` on success
 */
esp_err_t mp2660_get_fault(i2c_dev_t *dev, mp2660_fault_t* fault);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif
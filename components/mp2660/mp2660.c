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
 * @file mp2660.c
 *
 * ESP-IDF driver for Monolithic Systems 5V USB, 500mA, I2C-Controlled Linear Charger with Power Path Management for Single-Cell Li-Ion Battery
 *
 * Copyright (c) 2023 Manuel Markwort <https://github.com/mmarkwort>\n
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_idf_lib_helpers.h>
#include "mp2660.h"

#define I2C_FREQ_HZ 400000 //!< 400kHz bus speed

#define MP2660_SOURCE_CONTROL_REG_ADDR 0x00 //!< Address of source control register
#define MP2660_POWER_ON_CONFIG_REG_ADDR 0x01 //!< Address of power on configuration register
#define MP2660_CURENT_CONTROL_REG_ADDR 0x02 //!< Address of current control register
#define MP2660_PRECHARGE_TERMINATION_CURRENT_REG_ADDR 0x03 //!< Address of precharge termination current register
#define MP2660_CHARGE_VOLTAGE_CONTROL_REG_ADDR 0x04 //!< Address of charge voltage control register
#define MP2660_CHARGE_TERMINATION_TIMER_CONTROL_REG_ADDR 0x05 //!< Address of charge termination timer control register
#define MP2660_MISC_OPERATION_CONTROL_REG_ADDR 0x06 //!< Address of misc operation control register
#define MP2660_SYSTEM_STATUS_REG_ADDR 0x07 //!< Address of system status register
#define MP2660_FAULT_REG_ADDR 0x08 //!< Address of fault register

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t mp2660_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = MP2660_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t mp2660_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t mp2660_read(i2c_dev_t *dev, uint8_t addr, void* data)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, addr, data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mp2660_write(i2c_dev_t *dev, uint8_t addr, void* data)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, addr, data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mp2660_get_input_source(i2c_dev_t *dev, mp2660_input_source_t* input_source)
{
    return mp2660_read(dev, MP2660_SOURCE_CONTROL_REG_ADDR, input_source);
}

esp_err_t mp2660_set_input_source(i2c_dev_t *dev, mp2660_input_source_t* input_source)
{
    return mp2660_write(dev, MP2660_SOURCE_CONTROL_REG_ADDR, input_source);
}

esp_err_t mp2660_get_pwr_on_conf(i2c_dev_t *dev, mp2660_power_on_conf_t* pwr_config)
{
    return mp2660_read(dev, MP2660_POWER_ON_CONFIG_REG_ADDR, pwr_config);
}

esp_err_t mp2660_set_pwr_on_conf(i2c_dev_t *dev, mp2660_power_on_conf_t* pwr_config)
{
    return mp2660_write(dev, MP2660_POWER_ON_CONFIG_REG_ADDR, pwr_config);
}

esp_err_t mp2660_get_chrg_current_ctrl(i2c_dev_t *dev, mp2660_charge_current_ctrl_t* chrg_current_ctrl)
{
    return mp2660_read(dev, MP2660_CURENT_CONTROL_REG_ADDR, chrg_current_ctrl);
}

esp_err_t mp2660_set_chrg_current_ctrl(i2c_dev_t *dev, mp2660_charge_current_ctrl_t* chrg_current_ctrl)
{
    return mp2660_write(dev, MP2660_CURENT_CONTROL_REG_ADDR, chrg_current_ctrl);
}

esp_err_t mp2660_get_pre_chrg_term_current(i2c_dev_t *dev, mp2660_pre_charge_term_current_t* pre_chrg_term_current)
{
   return mp2660_read(dev, MP2660_PRECHARGE_TERMINATION_CURRENT_REG_ADDR, pre_chrg_term_current);
}

esp_err_t mp2660_set_pre_chrg_term_current(i2c_dev_t *dev, mp2660_pre_charge_term_current_t* pre_chrg_term_current)
{
   return mp2660_write(dev, MP2660_PRECHARGE_TERMINATION_CURRENT_REG_ADDR, pre_chrg_term_current);
}

esp_err_t mp2660_get_chrg_voltage_control(i2c_dev_t *dev, mp2660_charge_voltage_ctrl_t* chrg_voltage_ctrl)
{
    return mp2660_read(dev, MP2660_CHARGE_VOLTAGE_CONTROL_REG_ADDR, chrg_voltage_ctrl);
}

esp_err_t mp2660_set_chrg_voltage_control(i2c_dev_t *dev, mp2660_charge_voltage_ctrl_t* chrg_voltage_ctrl)
{
    return mp2660_write(dev, MP2660_CHARGE_VOLTAGE_CONTROL_REG_ADDR, chrg_voltage_ctrl);
}

esp_err_t mp2660_get_chrg_term_timer_control(i2c_dev_t *dev, mp2660_charge_term_timer_ctrl_t* charge_term_timer_ctrl)
{
    return mp2660_read(dev, MP2660_CHARGE_TERMINATION_TIMER_CONTROL_REG_ADDR, charge_term_timer_ctrl);
}

esp_err_t mp2660_set_chrg_term_timer_control(i2c_dev_t *dev, mp2660_charge_term_timer_ctrl_t* charge_term_timer_ctrl)
{
    return mp2660_write(dev, MP2660_CHARGE_TERMINATION_TIMER_CONTROL_REG_ADDR, charge_term_timer_ctrl);
}

esp_err_t mp2660_get_misc_op_control(i2c_dev_t *dev, mp2660_misc_op_ctrl_t* misc_op_ctrl)
{
    return mp2660_read(dev, MP2660_MISC_OPERATION_CONTROL_REG_ADDR, misc_op_ctrl);
}

esp_err_t mp2660_set_misc_op_control(i2c_dev_t *dev, mp2660_misc_op_ctrl_t* misc_op_ctrl)
{
    return mp2660_write(dev, MP2660_MISC_OPERATION_CONTROL_REG_ADDR, misc_op_ctrl);
}

esp_err_t mp2660_get_sys_status(i2c_dev_t *dev, mp2660_sys_status_t* sys_status)
{
    return mp2660_read(dev, MP2660_SYSTEM_STATUS_REG_ADDR, sys_status);
}

esp_err_t mp2660_get_fault(i2c_dev_t *dev, mp2660_fault_t* fault)
{
    return mp2660_read(dev, MP2660_FAULT_REG_ADDR, fault);
}
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
 * @file tps63101x.c
 *
 * ESP-IDF driver for Texas Instruments TPS631012 and TPS631013 1.6-V to 5.5-V Input Voltage 1.5-A Buck-boost Converter with I2C
 *
 * Copyright (c) 2024 Manuel Markwort <https://github.com/mmarkwort>\n
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_idf_lib_helpers.h>
#include "tps63101x.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_FREQ_HZ 400000 //!< 400kHz bus speed

#define TPS63101X_CONTROL_1_REG_ADDR 0x02 //!< Address of Control 1 register
#define TPS63101X_VOUT_REG_ADDR 0x03 //!< Address of VOUT register
#define TPS63101X_CONTROL_2_REG_ADDR 0x05 //!< Address of Control 2 register

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t tps63101x_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = TPS63101X_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t tps63101x_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t tps63101x_read(i2c_dev_t *dev, uint8_t addr, void* data)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, addr, data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t tps63101x_write(i2c_dev_t *dev, uint8_t addr, void* data)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, addr, data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t tps63101x_get_control_1(i2c_dev_t *dev, tps63101x_control_1_t* control_1)
{
    return tps63101x_read(dev, TPS63101X_CONTROL_1_REG_ADDR, control_1);
}

esp_err_t tps63101x_set_control_1(i2c_dev_t *dev, tps63101x_control_1_t* control_1)
{
    return tps63101x_write(dev, TPS63101X_CONTROL_1_REG_ADDR, control_1);
}

esp_err_t tps63101x_get_vout(i2c_dev_t *dev, tps63101x_vout_t* vout)
{
    return tps63101x_read(dev, TPS63101X_VOUT_REG_ADDR, vout);
}

esp_err_t tps63101x_set_vout(i2c_dev_t *dev, tps63101x_vout_t* vout)
{
    return tps63101x_write(dev, TPS63101X_VOUT_REG_ADDR, vout);
}

esp_err_t tps63101x_get_control_2(i2c_dev_t *dev, tps63101x_control_2_t* control_2)
{
    return tps63101x_read(dev, TPS63101X_CONTROL_2_REG_ADDR, control_2);
}

esp_err_t tps63101x_set_control_2(i2c_dev_t *dev, tps63101x_control_2_t* control_2)
{
    return tps63101x_write(dev, TPS63101X_CONTROL_2_REG_ADDR, control_2);
}

esp_err_t tps63101x_reset(i2c_dev_t *dev)
{
    esp_err_t err;

    tps63101x_control_1_t control_1;
    tps63101x_vout_t vout;
    tps63101x_control_2_t control_2;

    control_1.register_data.reg = TPS63101X_CONTROL_1_DEFAULT;
    vout.register_data.reg = TPS63101X_VOUT_DEFAULT;
    control_2.register_data.reg = TPS63101X_CONTROL_2_DEFAULT;

    err = tps63101x_set_control_1(dev, &control_1);
    if(err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(1500));

    err = tps63101x_set_control_2(dev, &control_2);
    if(err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(1500));

    err = tps63101x_set_vout(dev, &vout);
    if(err != ESP_OK)
    {
        return err;
    }

    return err;
}

uint8_t tps63101x_to_register_voltage(float voltage)
{
    if(voltage < 1.0f || voltage > 5.5f)
    {
        return 0xff;
    }

    return (voltage - 1.0f) / 0.025;
}
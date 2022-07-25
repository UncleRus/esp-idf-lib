/*
 * Copyright (c) 2022 Jose Manuel Perez <user@your.dom.ain>
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

/*
 * this file is supposed to conform the code style.
 */

#include <freertos/FreeRTOS.h>
#include <esp_idf_lib_helpers.h>
#include <esp_err.h>
#include <esp_log.h>

#include "lc709203f.h"

#define LC709203F_I2C_ADDR        0x0B   ///< LC709203F default i2c address
#define LC709003F_I2C_MAX_FREQ_HZ 400000 ///< 400kHz

#define LC709203F_INIT_RSOC_VAL  0xAA55 ///< Value to init RSOC
#define LC709203F_CRC_POLYNOMIAL 0x07   /// Polynomial to calculare CRC-8-ATM

// static char *tag = "lc709203f";

// clang-format off
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
// clang-format on

static uint8_t s_lc709203f_calc_crc(uint8_t *data, size_t data_len)
{
    uint8_t crc = 0;

    for (size_t j = data_len; j; --j)
    {
        crc ^= *data++;

        for (size_t i = 8; i; --i)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ LC709203F_CRC_POLYNOMIAL : (crc << 1);
        }
    }

    return crc;
}

inline static esp_err_t s_i2c_dev_read_word(i2c_dev_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t read_data[6] = { 0 };
    uint8_t crc = 0;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, read_data + 3, 3));
    I2C_DEV_GIVE_MUTEX(dev);

    read_data[0] = dev->addr << 1;
    read_data[1] = reg;
    read_data[2] = read_data[0] | 0x01;

    crc = s_lc709203f_calc_crc(read_data, 5);

    if (crc != read_data[5])
    {
        return ESP_ERR_INVALID_CRC;
    }

    if (value)
    {
        *value = read_data[3] | (read_data[4] << 8);
    }

    return ESP_OK;
}

inline static esp_err_t s_i2c_dev_write_reg_word(i2c_dev_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t write_data[5];
    write_data[0] = dev->addr << 1;
    write_data[1] = reg;
    write_data[2] = value & 0xFF;
    write_data[3] = value >> 8;
    write_data[4] = s_lc709203f_calc_crc(write_data, 4);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, write_data + 2, 3));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t lc709203f_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->addr = LC709203F_I2C_ADDR;
    dev->port = port;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = LC709003F_I2C_MAX_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t lc709203f_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t lc709203f_before_rsoc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_BEFORE_RSOC, LC709203F_INIT_RSOC_VAL);
}

esp_err_t lc709203f_initial_rsoc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_INITIAL_RSOC, LC709203F_INIT_RSOC_VAL);
}

esp_err_t lc709203f_get_alarm_low_rsoc(i2c_dev_t *dev, uint8_t *rsoc)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_ALARM_LOW_RSOC, (uint16_t *)rsoc);
}

esp_err_t lc709203f_get_alarm_low_voltage(i2c_dev_t *dev, uint16_t *voltage)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_ALARM_LOW_VOLTAGE, voltage);
}

esp_err_t lc709203f_get_apa(i2c_dev_t *dev, uint8_t *apa)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_APA, (uint16_t *)apa);
}

esp_err_t lc709203f_get_apt(i2c_dev_t *dev, uint16_t *apt)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_APT, apt);
}

esp_err_t lc709203f_get_battery_profile(i2c_dev_t *dev, lc709203f_battery_profile_t *profile)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_CHANGE_PARAMETER, (uint16_t *)profile);
}

esp_err_t lc709203f_get_battery_profile_code(i2c_dev_t *dev, uint16_t *code)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_NUM_PARAMETER, code);
}

esp_err_t lc709203f_get_cell_ite(i2c_dev_t *dev, uint16_t *ite)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_CELL_ITE, ite);
}

esp_err_t lc709203f_get_cell_temperature(i2c_dev_t *dev, float *temperature)
{
    CHECK_ARG(dev);

    uint16_t temp = 0;

    esp_err_t ret = s_i2c_dev_read_word(dev, LC709203F_REG_CELL_TEMPERATURE, &temp);

    if (ret == ESP_OK)
    {
        *temperature = temp / 10.0;
    }

    return ret;
}

esp_err_t lc709203f_get_cell_temperature_celsius(i2c_dev_t *dev, float *temperature)
{
    esp_err_t ret = lc709203f_get_cell_temperature(dev, temperature);

    if (ret == ESP_OK)
    {
        *temperature = *temperature - 273;
    }

    return ret;
}

esp_err_t lc709203f_get_cell_voltage(i2c_dev_t *dev, uint16_t *voltage)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_CELL_VOLTAGE, voltage);
}

esp_err_t lc709203f_get_current_direction(i2c_dev_t *dev, lc709203f_direction_t *direction)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_CELL_VOLTAGE, (uint16_t *)direction);
}

esp_err_t lc709203f_get_ic_version(i2c_dev_t *dev, uint16_t *ic_version)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_IC_VERSION, ic_version);
}

esp_err_t lc709203f_get_power_mode(i2c_dev_t *dev, lc709203f_power_mode_t *mode)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_IC_POWER_MODE, (uint16_t *)mode);
}

esp_err_t lc709203f_get_rsoc(i2c_dev_t *dev, uint16_t *rsoc)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_RSOC, rsoc);
}

esp_err_t lc709203f_get_temp_mode(i2c_dev_t *dev, lc709203f_temp_mode_t *mode)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_STATUS_BIT, (uint16_t *)mode);
}

esp_err_t lc709203f_get_thermistor_b(i2c_dev_t *dev, uint16_t *value)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_THERMISTOR_B, value);
}

esp_err_t lc709203f_set_alarm_low_rsoc(i2c_dev_t *dev, uint8_t rsoc)
{
    CHECK_ARG(dev);
    CHECK_ARG(rsoc <= 100);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_ALARM_LOW_RSOC, (uint16_t)rsoc);
}

esp_err_t lc709203f_set_alarm_low_voltage(i2c_dev_t *dev, uint16_t voltage)
{
    CHECK_ARG(dev);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_ALARM_LOW_VOLTAGE, voltage);
}

esp_err_t lc709203f_set_apa(i2c_dev_t *dev, uint8_t apa)
{
    CHECK_ARG(dev);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_APA, (uint16_t)apa);
}

esp_err_t lc709203f_set_apt(i2c_dev_t *dev, uint16_t apt)
{
    CHECK_ARG(dev);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_APT, apt);
}

esp_err_t lc709203f_set_battery_profile(i2c_dev_t *dev, lc709203f_battery_profile_t profile)
{
    CHECK_ARG(dev);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_CHANGE_PARAMETER, (uint16_t)profile);
}

esp_err_t lc709203f_set_cell_temperature(i2c_dev_t *dev, float temperature)
{
    uint16_t temp = (uint16_t)(temperature * 10);

    CHECK_ARG(dev);
    CHECK_ARG(temp >= 0x09e4 && temperature <= 0x0D04);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_CELL_TEMPERATURE, temp);
}

esp_err_t lc709203f_set_cell_temperature_celsius(i2c_dev_t *dev, float temperature)
{
    CHECK_ARG(dev);

    return lc709203f_set_cell_temperature(dev, temperature + 273);
}

esp_err_t lc709203f_set_current_direction(i2c_dev_t *dev, lc709203f_direction_t direction)
{
    CHECK_ARG(dev);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_CURRENT_DIRECTION, (uint16_t)direction);
}

esp_err_t lc709203f_set_power_mode(i2c_dev_t *dev, lc709203f_power_mode_t mode)
{
    CHECK_ARG(dev);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_IC_POWER_MODE, (uint16_t)mode);
}

esp_err_t lc709203f_set_temp_mode(i2c_dev_t *dev, lc709203f_temp_mode_t mode)
{
    CHECK_ARG(dev);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_STATUS_BIT, (uint16_t)mode);
}

esp_err_t lc709203f_set_thermistor_b(i2c_dev_t *dev, uint16_t value)
{
    CHECK_ARG(dev);

    return s_i2c_dev_write_reg_word(dev, LC709203F_REG_THERMISTOR_B, value);
}

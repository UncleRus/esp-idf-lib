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

#define LC709203F_REG_BEFORE_RSOC       0x04 ///< Initialize before RSOC
#define LC709203F_REG_THERMISTOR_B      0x06 ///< Read/write thermistor B
#define LC709203F_REG_INITIAL_RSOC      0x07 ///< Initialize RSOC calculation
#define LC709203F_REG_CELL_TEMPERATURE  0x08 ///< Read/write batt temperature
#define LC709203F_REG_CELL_VOLTAGE      0x09 ///< Read batt voltage
#define LC709203F_REG_CURRENT_DIRECTION 0x0A ///< Read/write current direction
#define LC709203F_REG_APA               0x0B ///< Read/write Adjustment Pack Application
#define LC709203F_REG_APT               0x0C ///< Read/write Adjustment Pack Thermistor
#define LC709203F_REG_RSOC              0x0D ///< Read state of charge
#define LC709203F_REG_CELL_ITE          0x0F ///< Read batt indicator to empty
#define LC709203F_REG_IC_VERSION        0x11 ///< Read IC version
#define LC709203F_REG_CHANGE_PARAMETER  0x12 ///< Set the battery profile
#define LC709203F_REG_ALARM_LOW_RSOC    0x13 ///< Alarm on percent threshold
#define LC709203F_REG_ALARM_LOW_VOLTAGE 0x14 ///< Alarm on voltage threshold
#define LC709203F_REG_IC_POWER_MODE     0x15 ///< Sets sleep/power mode
#define LC709203F_REG_STATUS_BIT        0x16 ///< Temperature obtaining method
#define LC709203F_REG_NUM_PARAMETER     0x1A ///< Batt profile code

static const uint16_t s_lc709203f_init_rsoc_val = 0xAA55; ///< Value to init RSOC

static char *tag = "lc709203f";

#define CHECK(x)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        esp_err_t __;                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                        \
            return __;                                                                                                 \
    } while (0)

#define CHECK_ARG(VAL)                                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(VAL))                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                \
    } while (0)

inline static esp_err_t s_i2c_dev_read_word(i2c_dev_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t reg_val[] = { 0, 0 };

    CHECK(i2c_dev_read_reg(dev, reg, reg_val, 2));

    if (value)
    {
        *value = reg_val[0] | (reg_val[1] << 8);
    }

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

    uint8_t reg = LC709203F_REG_BEFORE_RSOC;
    return i2c_dev_write(dev, &reg, 1, &s_lc709203f_init_rsoc_val, 2);
}

esp_err_t lc709203f_initial_rsoc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_write(dev, LC709203F_REG_INITIAL_RSOC, 1, &s_lc709203f_init_rsoc_val, 2);
}

esp_err_t lc709203f_get_alarm_low_rsoc(i2c_dev_t *dev, uint8_t *rsoc)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_ALARM_LOW_RSOC, rsoc);
}

esp_err_t lc709203f_get_alarm_low_voltage(i2c_dev_t *dev, uint16_t *voltage)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_ALARM_LOW_VOLTAGE, voltage);
}

esp_err_t lc709203f_get_apa(i2c_dev_t *dev, uint8_t *apa)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_APA, apa);
}

esp_err_t lc709203f_get_apt(i2c_dev_t *dev, uint8_t *apt)
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

esp_err_t lc709203f_get_cell_temperature(i2c_dev_t *dev, uint16_t *temperature)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_CELL_TEMPERATURE, temperature);
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

    return s_i2c_dev_read_word(dev, LC709203F_REG_IC_POWER_MODE, mode);
}

esp_err_t lc709203f_get_rsoc(i2c_dev_t *dev, uint16_t *rsoc)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_RSOC, rsoc);
}

esp_err_t lc709203f_get_temp_mode(i2c_dev_t *dev, lc709203f_temp_mode_t *mode)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_STATUS_BIT, mode);
}

esp_err_t lc709203f_get_thermistor_b(i2c_dev_t *dev, uint16_t *value)
{
    CHECK_ARG(dev);

    return s_i2c_dev_read_word(dev, LC709203F_REG_THERMISTOR_B, value);
}

esp_err_t lc709203f_set_alarm_low_rsoc(i2c_dev_t *dev, uint8_t rsoc)
{
    CHECK_ARG(dev);
    CHECK_ARG(rsoc >= 0 && rsoc <= 100);
    uint8_t reg = LC709203F_REG_ALARM_LOW_RSOC;
    return i2c_dev_write(dev, &reg, 1, (uint16_t)&rsoc, 2);
}

esp_err_t lc709203f_set_alarm_low_voltage(i2c_dev_t *dev, uint16_t voltage)
{
    CHECK_ARG(dev);

    uint8_t reg = LC709203F_REG_ALARM_LOW_VOLTAGE;
    return i2c_dev_write(dev, &reg, 1, &voltage, 2);
}

esp_err_t lc709203f_set_apa(i2c_dev_t *dev, uint8_t apa)
{
    CHECK_ARG(dev);

    uint8_t reg = LC709203F_REG_APA;
    return i2c_dev_write(dev, &reg, 1, (uint16_t *)&apa, 2);
}

esp_err_t lc709203f_set_apt(i2c_dev_t *dev, uint16_t apt)
{
    CHECK_ARG(dev);

    uint8_t reg = LC709203F_REG_APA;
    return i2c_dev_write(dev, &reg, 1, &apt, 2);
}

esp_err_t lc709203f_set_battery_profile(i2c_dev_t *dev, lc709203f_battery_profile_t profile)
{
    CHECK_ARG(dev);

    uint8_t reg = LC709203F_REG_CHANGE_PARAMETER;
    return i2c_dev_write(dev, &reg, 1, (uint16_t *)&profile, 2);
}

esp_err_t lc709203f_set_cell_temperature(i2c_dev_t *dev, uint16_t temperature)
{
    CHECK_ARG(dev);
    CHECK_ARG(temperature >= 0x09e4 && temperature <= 0x0D04);
    
    uint8_t reg = LC709203F_REG_CELL_TEMPERATURE;
    return i2c_dev_write(dev, &reg, 1, &temperature, 2);
}

esp_err_t lc709203f_set_current_direction(i2c_dev_t *dev, lc709203f_direction_t direction)
{
    CHECK_ARG(dev);

    uint8_t reg = LC709203F_REG_CURRENT_DIRECTION;
    return i2c_dev_write(dev, &reg, 1, (uint16_t *)&direction, 2);
}

esp_err_t lc709203f_set_power_mode(i2c_dev_t *dev, lc709203f_power_mode_t mode)
{
    CHECK_ARG(dev);

    uint8_t reg = LC709203F_REG_IC_POWER_MODE;
    return i2c_dev_write(dev, &reg, 1, (uint16_t *)&mode, 2);
}

esp_err_t lc709203f_set_temp_mode(i2c_dev_t *dev, lc709203f_temp_mode_t mode)
{
    CHECK_ARG(dev);

    uint8_t reg = LC709203F_REG_STATUS_BIT;
    return i2c_dev_write(dev, &reg, 1, (uint16_t *)&mode, 2);
}

esp_err_t lc709203f_set_thermistor_b(i2c_dev_t *dev, uint16_t value)
{
    CHECK_ARG(dev);

    uint8_t reg = LC709203F_REG_THERMISTOR_B;
    return i2c_dev_write(dev, &reg, 1, (uint16_t *)&value, 2);
}

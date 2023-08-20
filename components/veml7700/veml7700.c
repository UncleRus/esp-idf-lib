/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2022 Marc Luehr <marcluehr@gmail.com>
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

#define VEML7700_COMMAND_CODE_ALS_CONF_0   (0)
#define VEML7700_COMMAND_CODE_ALS_WH       (1)
#define VEML7700_COMMAND_CODE_ALS_WL       (2)
#define VEML7700_COMMAND_CODE_POWER_SAVING (3)
#define VEML7700_COMMAND_CODE_ALS          (4)
#define VEML7700_COMMAND_CODE_WHITE        (5)
#define VEML7700_COMMAND_CODE_ALS_INT      (6)

#define VEML7700_GAIN_MASK  (0x1800)
#define VEML7700_GAIN_SHIFT (11)

#define VEML7700_INTEGRATION_TIME_MASK  (0x03C0)
#define VEML7700_INTEGRATION_TIME_SHIFT (6)

#define VEML7700_PERSISTENCE_PROTECTION_MASK  (0x0030)
#define VEML7700_PERSISTENCE_PROTECTION_SHIFT (4)

#define VEML7700_INTERRUPT_ENABLE_MASK  (0x0002)
#define VEML7700_INTERRUPT_ENABLE_SHIFT (1)

#define VEML7700_SHUTDOWN_MASK  (0x0001)
#define VEML7700_SHUTDOWN_SHIFT (0)

#define VEML7700_POWER_SAVING_MODE_MASK  (0x0060)
#define VEML7700_POWER_SAVING_MODE_SHIFT (1)

#define VEML7700_POWER_SAVING_MODE_ENABLE_MASK  (0x0001)
#define VEML7700_POWER_SAVING_MODE_ENABLE_SHIFT (0)

#define VEML7700_INTERRUPT_STATUS_LOW_MASK   (0x8000)
#define VEML7700_INTERRUPT_STATUS_LOW_SHIFT  (15)
#define VEML7700_INTERRUPT_STATUS_HIGH_MASK  (0x4000)
#define VEML7700_INTERRUPT_STATUS_HIGH_SHIFT (14)

#define VEML7700_RESOLUTION_800MS_IT_GAIN_2     (36)
#define VEML7700_RESOLUTION_800MS_IT_GAIN_2_DIV (10000)

/**
 * @file veml7700.c
 *
 * ESP-IDF driver for VEML7700 brightness sensors for I2C-bus
 *
 * Copyright (c) 2022 Marc Luehr <marcluehr@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "veml7700.h"

#define I2C_FREQ_HZ (100000)

#define CHECK(x)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        esp_err_t __;                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                        \
            return __;                                                                                                 \
    }                                                                                                                  \
    while (0)
#define CHECK_ARG(VAL)                                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(VAL))                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                \
    }                                                                                                                  \
    while (0)

static esp_err_t read_port(i2c_dev_t *dev, uint8_t command_code, uint16_t *data)
{
    CHECK_ARG(dev);

    I2C_DEV_CHECK(dev, i2c_dev_read(dev, &command_code, 1, data, 2));

    return ESP_OK;
}

static esp_err_t write_port(i2c_dev_t *dev, uint8_t command_code, uint16_t data)
{
    CHECK_ARG(dev);

    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &command_code, 1, &data, 2));

    return ESP_OK;
}

static uint32_t resolution(veml7700_config_t *config)
{
    CHECK_ARG(config);

    uint32_t resolution = VEML7700_RESOLUTION_800MS_IT_GAIN_2;
    switch (config->gain)
    {
        case VEML7700_GAIN_1:
            resolution = resolution * 2;
            break;
        case VEML7700_GAIN_DIV_4:
            resolution = resolution * 8;
            break;
        case VEML7700_GAIN_DIV_8:
            resolution = resolution * 16;
            break;
    }
    switch (config->integration_time)
    {
        case VEML7700_INTEGRATION_TIME_400MS:
            resolution = resolution * 2;
            break;
        case VEML7700_INTEGRATION_TIME_200MS:
            resolution = resolution * 4;
            break;
        case VEML7700_INTEGRATION_TIME_100MS:
            resolution = resolution * 8;
            break;
        case VEML7700_INTEGRATION_TIME_50MS:
            resolution = resolution * 16;
            break;
        case VEML7700_INTEGRATION_TIME_25MS:
            resolution = resolution * 32;
            break;
    }
    return resolution;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t veml7700_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = VEML7700_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t veml7700_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t veml7700_probe(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    /* use write request since read request causes a timeout;
     * just doing a read is not intended to use by the chip,
     * it is waiting for a command code
     */
    I2C_DEV_TAKE_MUTEX(dev);
    esp_err_t err = i2c_dev_probe(dev, I2C_DEV_WRITE);
    I2C_DEV_GIVE_MUTEX(dev);
    return err;
}

esp_err_t veml7700_set_config(i2c_dev_t *dev, veml7700_config_t *config)
{
    CHECK_ARG(dev);
    CHECK_ARG(config);

    uint16_t config_data = 0;
    config_data |= config->gain << VEML7700_GAIN_SHIFT;
    config_data |= config->integration_time << VEML7700_INTEGRATION_TIME_SHIFT;
    config_data |= config->persistence_protect << VEML7700_PERSISTENCE_PROTECTION_SHIFT;
    config_data |= config->interrupt_enable << VEML7700_INTERRUPT_ENABLE_SHIFT;
    config_data |= config->shutdown << VEML7700_SHUTDOWN_SHIFT;

    uint16_t power_saving_data = 0;
    power_saving_data |= config->power_saving_mode << VEML7700_POWER_SAVING_MODE_SHIFT;
    power_saving_data |= config->power_saving_enable << VEML7700_POWER_SAVING_MODE_ENABLE_SHIFT;
    
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_port(dev, VEML7700_COMMAND_CODE_ALS_CONF_0, config_data));
    I2C_DEV_CHECK(dev, write_port(dev, VEML7700_COMMAND_CODE_ALS_WH, config->threshold_high));
    I2C_DEV_CHECK(dev, write_port(dev, VEML7700_COMMAND_CODE_ALS_WL, config->threshold_low));
    I2C_DEV_CHECK(dev, write_port(dev, VEML7700_COMMAND_CODE_POWER_SAVING, power_saving_data));
    I2C_DEV_GIVE_MUTEX(dev);
    return ESP_OK;
}

esp_err_t veml7700_get_config(i2c_dev_t *dev, veml7700_config_t *config)
{
    CHECK_ARG(dev);
    CHECK_ARG(config);

    uint16_t config_data = 0;
    uint16_t power_saving_data = 0;
    
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_port(dev, VEML7700_COMMAND_CODE_ALS_CONF_0, &config_data));
    I2C_DEV_CHECK(dev, read_port(dev, VEML7700_COMMAND_CODE_ALS_WH, &(config->threshold_high)));
    I2C_DEV_CHECK(dev, read_port(dev, VEML7700_COMMAND_CODE_ALS_WL, &(config->threshold_low)));
    I2C_DEV_CHECK(dev, read_port(dev, VEML7700_COMMAND_CODE_POWER_SAVING, &power_saving_data));
    I2C_DEV_GIVE_MUTEX(dev);
    
    config->gain = (config_data & VEML7700_GAIN_MASK) >> VEML7700_GAIN_SHIFT;
    config->integration_time = (config_data & VEML7700_INTEGRATION_TIME_MASK) 
        >> VEML7700_INTEGRATION_TIME_SHIFT;
    config->integration_time = (config_data & VEML7700_PERSISTENCE_PROTECTION_MASK) 
        >> VEML7700_PERSISTENCE_PROTECTION_SHIFT;
    config->integration_time = (config_data & VEML7700_INTERRUPT_ENABLE_MASK) 
        >> VEML7700_INTERRUPT_ENABLE_SHIFT;
    config->integration_time = (config_data & VEML7700_SHUTDOWN_MASK) 
        >> VEML7700_SHUTDOWN_SHIFT;

    config->power_saving_mode = (power_saving_data & VEML7700_POWER_SAVING_MODE_MASK)
        >> VEML7700_POWER_SAVING_MODE_SHIFT;
    config->power_saving_enable = (power_saving_data & VEML7700_POWER_SAVING_MODE_ENABLE_MASK)
        >> VEML7700_POWER_SAVING_MODE_ENABLE_SHIFT;

    return ESP_OK;
}

esp_err_t veml7700_get_ambient_light(i2c_dev_t *dev, veml7700_config_t *config, uint32_t *value)
{
    CHECK_ARG(dev);
    CHECK_ARG(config);

    uint16_t raw_value = 0;
    CHECK(read_port(dev, VEML7700_COMMAND_CODE_ALS, &raw_value));

    *value = (raw_value * resolution(config)) / VEML7700_RESOLUTION_800MS_IT_GAIN_2_DIV;
    return ESP_OK;
}

esp_err_t veml7700_get_white_channel(i2c_dev_t *dev, veml7700_config_t *config, uint32_t *value)
{
    CHECK_ARG(dev);
    CHECK_ARG(config);

    uint16_t raw_value = 0;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_port(dev, VEML7700_COMMAND_CODE_WHITE, &raw_value));
    I2C_DEV_GIVE_MUTEX(dev);

    *value = (raw_value * resolution(config)) / VEML7700_RESOLUTION_800MS_IT_GAIN_2_DIV;
    return ESP_OK;
}

esp_err_t veml7700_get_interrupt_status(i2c_dev_t *dev, bool *low_threshold, bool *high_threshold)
{
    CHECK_ARG(dev);

    uint16_t interrupt_status = 0;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_port(dev, VEML7700_COMMAND_CODE_ALS_INT, &interrupt_status));
    I2C_DEV_GIVE_MUTEX(dev);

    *high_threshold = interrupt_status & VEML7700_INTERRUPT_STATUS_HIGH_MASK;
    *low_threshold = interrupt_status & VEML7700_INTERRUPT_STATUS_LOW_MASK;
    return ESP_OK;
}

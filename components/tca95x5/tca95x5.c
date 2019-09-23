/**
 * @file tca95x5.c
 *
 * ESP-IDF driver for TCA9535/TCA9555 remote 16-bit I/O expanders for I2C-bus
 *
 * Copyright (C) 2019 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_idf_lib_helpers.h>
#include "tca95x5.h"

#define I2C_FREQ_HZ 400000

#define REG_IN0   0x00
#define REG_OUT0  0x02
#define REG_CONF0 0x06

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

static esp_err_t read_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, val, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t write_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t val)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &val, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t pcf8574_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    CHECK_ARG(addr & TCA95X5_I2C_ADDR_BASE);

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t pcf8574_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t tca95x5_port_get_mode(i2c_dev_t *dev, uint16_t *mode)
{
    return read_reg_16(dev, REG_CONF0, mode);
}

esp_err_t tca95x5_port_set_mode(i2c_dev_t *dev, uint16_t mode)
{
    return write_reg_16(dev, REG_CONF0, mode);
}

esp_err_t tca95x5_port_read(i2c_dev_t *dev, uint16_t *val)
{
    return read_reg_16(dev, REG_IN0, val);
}

esp_err_t tca95x5_port_write(i2c_dev_t *dev, uint16_t val)
{
    return write_reg_16(dev, REG_OUT0, val);
}

esp_err_t tca95x5_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t *val)
{
    uint16_t v;
    CHECK(read_reg_16(dev, REG_IN0, &v));
    *val = v & BV(pin) ? 1 : 0;

    return ESP_OK;
}

esp_err_t tca95x5_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val)
{
    uint16_t v;
    CHECK(read_reg_16(dev, REG_OUT0, &v));
    v = (v & ~BV(pin)) | (val ? BV(pin) : 0);
    return write_reg_16(dev, REG_OUT0, v);
}


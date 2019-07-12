/**
 * @file mcp23017.c
 *
 * ESP-IDF driver for I2C 16 bit GPIO expander MCP23017
 *
 * Copyright (C) 2018 Ruslan V. Uss (https://github.com/UncleRus)
 * BSD Licensed as described in the file LICENSE
 */
#include "mcp23017.h"
#include <esp_log.h>

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 1.7Mhz

#define REG_IODIRA   0x00
#define REG_IODIRB   0x01
#define REG_IPOLA    0x02
#define REG_IPOLB    0x03
#define REG_GPINTENA 0x04
#define REG_GPINTENB 0x05
#define REG_DEFVALA  0x06
#define REG_DEFVALB  0x07
#define REG_INTCONA  0x08
#define REG_INTCONB  0x09
#define REG_IOCON    0x0A
#define REG_GPPUA    0x0C
#define REG_GPPUB    0x0D
#define REG_INTFA    0x0E
#define REG_INTFB    0x0F
#define REG_INTCAPA  0x10
#define REG_INTCAPB  0x11
#define REG_GPIOA    0x12
#define REG_GPIOB    0x13
#define REG_OLATA    0x14
#define REG_OLATB    0x15

#define BIT_IOCON_INTPOL 1
#define BIT_IOCON_ODR    2
#define BIT_IOCON_HAEN   3
#define BIT_IOCON_DISSLW 4
#define BIT_IOCON_SEQOP  5
#define BIT_IOCON_MIRROR 6
#define BIT_IOCON_BANK   7

static const char *TAG = "MCP23017";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

static esp_err_t read_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK_ARG(dev);
    CHECK_ARG(val);

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

static esp_err_t write_reg_bit_16(i2c_dev_t *dev, uint8_t reg, bool val, uint8_t bit)
{
    CHECK_ARG(dev);

    uint16_t buf;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, &buf, 2));
    buf = (buf & ~BV(bit)) | (val ? BV(bit) : 0);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &buf, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t read_reg_bit_16(i2c_dev_t *dev, uint8_t reg, bool *val, uint8_t bit)
{
    uint16_t buf;

    CHECK(read_reg_16(dev, reg, &buf));

    *val = (buf & BV(bit)) >> bit;

    return ESP_OK;
}

static esp_err_t read_reg_bit_8(i2c_dev_t *dev, uint8_t reg, bool *val, uint8_t bit)
{
    CHECK_ARG(dev);
    CHECK_ARG(val);

    uint8_t buf;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, &buf, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *val = (buf & BV(bit)) >> bit;

    return ESP_OK;
}

static esp_err_t write_reg_bit_8(i2c_dev_t *dev, uint8_t reg, bool val, uint8_t bit)
{
    CHECK_ARG(dev);

    uint8_t buf;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, &buf, 1));
    buf = (buf & ~BV(bit)) | (val ? BV(bit) : 0);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &buf, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mcp23017_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr < MCP23017_I2C_ADDR_BASE || addr > MCP23017_I2C_ADDR_BASE + 7)
    {
        ESP_LOGE(TAG, "Invalid device address: 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;

    return i2c_dev_create_mutex(dev);
}

esp_err_t mcp23017_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t mcp23017_setup_hw_addr(i2c_dev_t *dev, bool enable, uint8_t new_addr)
{
    CHECK(write_reg_bit_8(dev, REG_IOCON, enable, BIT_IOCON_HAEN));
    dev->addr = enable ? new_addr : MCP23017_I2C_ADDR_BASE;

    return ESP_OK;
}

esp_err_t mcp23017_get_int_out_mode(i2c_dev_t *dev, mcp23017_int_out_mode_t *mode)
{
    CHECK_ARG(mode);

    bool buf;
    CHECK(read_reg_bit_8(dev, REG_IOCON, &buf, BIT_IOCON_ODR));
    if (buf)
    {
        *mode = MCP23017_OPEN_DRAIN;
        return ESP_OK;
    }
    CHECK(read_reg_bit_8(dev, REG_IOCON, &buf, BIT_IOCON_INTPOL));
    *mode = buf ? MCP23017_ACTIVE_HIGH : MCP23017_ACTIVE_LOW;

    return ESP_OK;
}

esp_err_t mcp23017_set_int_out_mode(i2c_dev_t *dev, mcp23017_int_out_mode_t mode)
{
    if (mode == MCP23017_OPEN_DRAIN)
        return write_reg_bit_8(dev, REG_IOCON, true, BIT_IOCON_ODR);

    return write_reg_bit_8(dev, REG_IOCON, mode == MCP23017_ACTIVE_HIGH, BIT_IOCON_INTPOL);
}

esp_err_t mcp23017_port_get_mode(i2c_dev_t *dev, uint16_t *val)
{
    return read_reg_16(dev, REG_IODIRA, val);
}

esp_err_t mcp23017_port_set_mode(i2c_dev_t *dev, uint16_t val)
{
    return write_reg_16(dev, REG_IODIRA, val);
}

esp_err_t mcp23017_port_get_pullup(i2c_dev_t *dev, uint16_t *val)
{
    return read_reg_16(dev, REG_GPPUA, val);
}

esp_err_t mcp23017_port_set_pullup(i2c_dev_t *dev, uint16_t val)
{
    return write_reg_16(dev, REG_GPPUA, val);
}

esp_err_t mcp23017_port_read(i2c_dev_t *dev, uint16_t *val)
{
    return read_reg_16(dev, REG_GPIOA, val);
}

esp_err_t mcp23017_port_write(i2c_dev_t *dev, uint16_t val)
{
    return write_reg_16(dev, REG_GPIOA, val);
}

esp_err_t mcp23017_get_mode(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_mode_t *mode)
{
    CHECK_ARG(mode);

    bool buf;
    CHECK(read_reg_bit_16(dev, REG_IODIRA, &buf, pin));
    *mode = buf ? MCP23017_GPIO_INPUT : MCP23017_GPIO_OUTPUT;

    return ESP_OK;
}

esp_err_t mcp23017_set_mode(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_mode_t mode)
{
    return write_reg_bit_16(dev, REG_IODIRA, mode, pin);
}

esp_err_t mcp23017_get_pullup(i2c_dev_t *dev, uint8_t pin, bool *enable)
{
    return read_reg_bit_16(dev, REG_GPPUA, enable, pin);
}

esp_err_t mcp23017_set_pullup(i2c_dev_t *dev, uint8_t pin, bool enable)
{
    return write_reg_bit_16(dev, REG_GPPUA, enable, pin);
}

esp_err_t mcp23017_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t *val)
{
    CHECK_ARG(val);

    bool buf;
    CHECK(read_reg_bit_16(dev, REG_GPIOA, &buf, pin));
    *val = buf ? 1 : 0;

    return ESP_OK;
}

esp_err_t mcp23017_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val)
{
    return write_reg_bit_16(dev, REG_GPIOA, val, pin);
}

esp_err_t mcp23017_port_set_interrupt(i2c_dev_t *dev, uint16_t mask, mcp23017_gpio_intr_t intr)
{
    CHECK_ARG(dev);

    uint16_t int_en;
    CHECK(read_reg_16(dev, REG_GPINTENA, &int_en));

    if (intr == MCP23017_INT_DISABLED)
    {
        // disable interrupts
        int_en &= ~mask;
        CHECK(write_reg_16(dev, REG_GPINTENA, int_en));

        return ESP_OK;
    }

    uint16_t int_con;
    CHECK(read_reg_16(dev, REG_INTCONA, &int_con));

    if (intr == MCP23017_INT_ANY_EDGE)
        int_con &= ~mask;
    else
    {
        int_con |= mask;

        uint16_t int_def;
        CHECK(read_reg_16(dev, REG_DEFVALA, &int_def));
        if (intr == MCP23017_INT_LOW_EDGE)
            int_def |= mask;
        else
            int_def &= ~mask;
        CHECK(write_reg_16(dev, REG_DEFVALA, int_def));
    }

    CHECK(write_reg_16(dev, REG_INTCONA, int_con));

    // enable interrupts
    int_en |= mask;
    CHECK(write_reg_16(dev, REG_GPINTENA, int_en));

    return ESP_OK;
}

esp_err_t mcp23017_set_interrupt(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_intr_t intr)
{
    return mcp23017_port_set_interrupt(dev, BV(pin), intr);
}

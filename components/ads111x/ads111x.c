/**
 * @file ads111x.c
 *
 * ESP-IDF driver for ADS1113/ADS1114/ADS1115 I2C ADC
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#include "ads111x.h"
#include <esp_log.h>

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp32

#define REG_CONVERSION 0
#define REG_CONFIG     1
#define REG_THRESH_L   2
#define REG_THRESH_H   3

#define COMP_QUE_OFFSET  1
#define COMP_QUE_MASK    0x03
#define COMP_LAT_OFFSET  2
#define COMP_LAT_MASK    0x01
#define COMP_POL_OFFSET  3
#define COMP_POL_MASK    0x01
#define COMP_MODE_OFFSET 4
#define COMP_MODE_MASK   0x01
#define DR_OFFSET        5
#define DR_MASK          0x07
#define MODE_OFFSET      8
#define MODE_MASK        0x01
#define PGA_OFFSET       9
#define PGA_MASK         0x07
#define MUX_OFFSET       12
#define MUX_MASK         0x07
#define OS_OFFSET        15
#define OS_MASK          0x01

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "ADS111x";

const float ads111x_gain_values[] = {
    [ADS111X_GAIN_6V144]   = 6.144,
    [ADS111X_GAIN_4V096]   = 4.096,
    [ADS111X_GAIN_2V048]   = 2.048,
    [ADS111X_GAIN_1V024]   = 1.024,
    [ADS111X_GAIN_0V512]   = 0.512,
    [ADS111X_GAIN_0V256]   = 0.256,
    [ADS111X_GAIN_0V256_2] = 0.256,
    [ADS111X_GAIN_0V256_3] = 0.256
};

static esp_err_t read_reg(i2c_dev_t *dev, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    esp_err_t res;
    if ((res = i2c_dev_read_reg(dev, reg, buf, 2)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read from register 0x%02x", reg);
        return res;
    }
    *val = (buf[0] << 8) | buf[1];

    return ESP_OK;
}

static esp_err_t write_reg(i2c_dev_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { val >> 8, val };
    esp_err_t res;
    if ((res = i2c_dev_write_reg(dev, reg, buf, 2)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not write 0x%04x to register 0x%02x", val, reg);
        return res;
    }

    return ESP_OK;
}

static esp_err_t read_conf_bits(i2c_dev_t *dev, uint8_t offs, uint16_t mask,
        uint16_t *bits)
{
    CHECK_ARG(dev);

    uint16_t val;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_CONFIG, &val));
    I2C_DEV_GIVE_MUTEX(dev);

    ESP_LOGD(TAG, "Got config value: 0x%04x", val);

    *bits = (val >> offs) & mask;

    return ESP_OK;
}

static esp_err_t write_conf_bits(i2c_dev_t *dev, uint16_t val, uint8_t offs,
        uint16_t mask)
{
    CHECK_ARG(dev);

    uint16_t old;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_CONFIG, &old));
    I2C_DEV_CHECK(dev, write_reg(dev, REG_CONFIG, (old & ~(mask << offs)) | (val << offs)));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

#define READ_CONFIG(OFFS, MASK, VAR) do { \
        CHECK_ARG(VAR); \
        uint16_t bits; \
        CHECK(read_conf_bits(dev, OFFS, MASK, &bits)); \
        *VAR = bits; \
        return ESP_OK; \
    } while(0)


///////////////////////////////////////////////////////////////////////////////

esp_err_t ads111x_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port,
        gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != ADS111X_ADDR_GND && addr != ADS111X_ADDR_VCC
            && addr != ADS111X_ADDR_SDA && addr != ADS111X_ADDR_SCL)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
    i2c_dev_create_mutex(dev);

    return ESP_OK;
}

esp_err_t ads111x_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t ads111x_is_busy(i2c_dev_t *dev, bool *busy)
{
    READ_CONFIG(OS_OFFSET, OS_MASK, busy);
}

esp_err_t ads111x_start_conversion(i2c_dev_t *dev)
{
    return write_conf_bits(dev, 1, OS_OFFSET, OS_MASK);
}

esp_err_t ads111x_get_value(i2c_dev_t *dev, int16_t *value)
{
    CHECK_ARG(dev);
    CHECK_ARG(value);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_CONVERSION, (uint16_t *)value));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ads111x_get_gain(i2c_dev_t *dev, ads111x_gain_t *gain)
{
    READ_CONFIG(PGA_OFFSET, PGA_MASK, gain);
}

esp_err_t ads111x_set_gain(i2c_dev_t *dev, ads111x_gain_t gain)
{
    return write_conf_bits(dev, gain, PGA_OFFSET, PGA_MASK);
}

esp_err_t ads111x_get_input_mux(i2c_dev_t *dev, ads111x_mux_t *mux)
{
    READ_CONFIG(MUX_OFFSET, MUX_MASK, mux);
}

esp_err_t ads111x_set_input_mux(i2c_dev_t *dev, ads111x_mux_t mux)
{
    return write_conf_bits(dev, mux, MUX_OFFSET, MUX_MASK);
}

esp_err_t ads111x_get_mode(i2c_dev_t *dev, ads111x_mode_t *mode)
{
    READ_CONFIG(MODE_OFFSET, MODE_MASK, mode);
}

esp_err_t ads111x_set_mode(i2c_dev_t *dev, ads111x_mode_t mode)
{
    return write_conf_bits(dev, mode, MODE_OFFSET, MODE_MASK);
}

esp_err_t ads111x_get_data_rate(i2c_dev_t *dev, ads111x_data_rate_t *rate)
{
    READ_CONFIG(DR_OFFSET, DR_MASK, rate);
}

esp_err_t ads111x_set_data_rate(i2c_dev_t *dev, ads111x_data_rate_t rate)
{
    return write_conf_bits(dev, rate, DR_OFFSET, DR_MASK);
}

esp_err_t ads111x_get_comp_mode(i2c_dev_t *dev, ads111x_comp_mode_t *mode)
{
    READ_CONFIG(COMP_MODE_OFFSET, COMP_MODE_MASK, mode);
}

esp_err_t ads111x_set_comp_mode(i2c_dev_t *dev, ads111x_comp_mode_t mode)
{
    return write_conf_bits(dev, mode, COMP_MODE_OFFSET, COMP_MODE_MASK);
}

esp_err_t ads111x_get_comp_polarity(i2c_dev_t *dev, ads111x_comp_polarity_t *polarity)
{
    READ_CONFIG(COMP_POL_OFFSET, COMP_POL_MASK, polarity);
}

esp_err_t ads111x_set_comp_polarity(i2c_dev_t *dev, ads111x_comp_polarity_t polarity)
{
    return write_conf_bits(dev, polarity, COMP_POL_OFFSET, COMP_POL_MASK);
}

esp_err_t ads111x_get_comp_latch(i2c_dev_t *dev, ads111x_comp_latch_t *latch)
{
    READ_CONFIG(COMP_LAT_OFFSET, COMP_LAT_MASK, latch);
}

esp_err_t ads111x_set_comp_latch(i2c_dev_t *dev, ads111x_comp_latch_t latch)
{
    return write_conf_bits(dev, latch, COMP_LAT_OFFSET, COMP_LAT_MASK);
}

esp_err_t ads111x_get_comp_queue(i2c_dev_t *dev, ads111x_comp_queue_t *queue)
{
    READ_CONFIG(COMP_QUE_OFFSET, COMP_QUE_MASK, queue);
}

esp_err_t ads111x_set_comp_queue(i2c_dev_t *dev, ads111x_comp_queue_t queue)
{
    return write_conf_bits(dev, queue, COMP_QUE_OFFSET, COMP_QUE_MASK);
}

esp_err_t ads111x_get_comp_low_thresh(i2c_dev_t *dev, int16_t *th)
{
    CHECK_ARG(dev);
    CHECK_ARG(th);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_THRESH_L, (uint16_t *)th));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ads111x_set_comp_low_thresh(i2c_dev_t *dev, int16_t th)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg(dev, REG_THRESH_L, th));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ads111x_get_comp_high_thresh(i2c_dev_t *dev, int16_t *th)
{
    CHECK_ARG(dev);
    CHECK_ARG(th);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_THRESH_H, (uint16_t *)th));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ads111x_set_comp_high_thresh(i2c_dev_t *dev, int16_t th)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg(dev, REG_THRESH_H, th));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

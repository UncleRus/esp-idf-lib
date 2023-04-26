/*
 * Copyright (c) 2023 Jose Manuel Perez <jmpmscorp@hotmail.com>
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

/**
 * @file sgm58031.c
 * @defgroup sgm58031 sgm58031
 * @{
 *
 * ESP-IDF driver for SGM58031 16-bit I2C ADC
 *
 * Copyright (c) 2023 Jose Manuel Perez <jmpmscorp@hotmail.com>
 */

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "sgm58031.h"

// #define I2C_FREQ_HZ (1000000) // Max 1MHz for esp32
#define I2C_FREQ_HZ (400000) // 400kHz

// SGM58031 registers
#define SGM58031_REG_CONVERSION (0x00)
#define SGM58031_REG_CONFIG     (0x01)
#define SGM58031_REG_THRESH_L   (0x02)
#define SGM58031_REG_THRESH_H   (0x03)
#define SGM58031_REG_CONFIG1    (0x04)
#define SGM58031_REG_CHIP_ID    (0x05)
#define SGM58031_REG_GN_TRIM1   (0x06)

/* REG CONFIG */
#define COMP_QUE_OFFSET  (1)
#define COMP_QUE_MASK    (0x03)
#define COMP_LAT_OFFSET  (2)
#define COMP_LAT_MASK    (0x01)
#define COMP_POL_OFFSET  (3)
#define COMP_POL_MASK    (0x01)
#define COMP_MODE_OFFSET (4)
#define COMP_MODE_MASK   (0x01)
#define DR_OFFSET        (5)
#define DR_MASK          (0x07)
#define MODE_OFFSET      (8)
#define MODE_MASK        (0x01)
#define PGA_OFFSET       (9)
#define PGA_MASK         (0x07)
#define MUX_OFFSET       (12)
#define MUX_MASK         (0x07)
#define OS_OFFSET        (15)
#define OS_MASK          (0x01)

/* REG CONFIG1*/
#define EXT_REF_OFFSET  (3)
#define EXT_REF_MASK    (0x01)
#define BUS_FLEX_OFFSET (4)
#define BUS_FLEX_MASK   (0x01)
#define INT_DIO_OFFSET  (5)
#define INT_DIO_MASK    (0x01)
#define BURNOUT_OFFSET  (6)
#define BURNOUT_MASK    (0x01)
#define DR_SEL_OFFSET   (7)
#define DR_SEL_MASK     (0x01)
#define PD_OFFSET       (8)
#define PD_MASK         (0x01)

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

static const char *TAG = "sgm58031";

const float sgm58031_gain_values[] = { [SGM58031_GAIN_6V144] = 6.144,
    [SGM58031_GAIN_4V096] = 4.096,
    [SGM58031_GAIN_2V048] = 2.048,
    [SGM58031_GAIN_1V024] = 1.024,
    [SGM58031_GAIN_0V512] = 0.512,
    [SGM58031_GAIN_0V256] = 0.256,
    [SGM58031_GAIN_0V256_2] = 0.256,
    [SGM58031_GAIN_0V256_3] = 0.256 };

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

static esp_err_t read_conf_bits(i2c_dev_t *dev, uint8_t config_reg, uint8_t offs, uint16_t mask, uint16_t *bits)
{
    CHECK_ARG(dev);
    CHECK_ARG((config_reg == SGM58031_REG_CONFIG) || (config_reg == SGM58031_REG_CONFIG1));

    uint16_t val;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, config_reg, &val));
    I2C_DEV_GIVE_MUTEX(dev);

    ESP_LOGD(TAG, "read config reg 0x%02x value: 0x%04x", config_reg, val);

    *bits = (val >> offs) & mask;

    return ESP_OK;
}

static esp_err_t write_conf_bits(i2c_dev_t *dev, uint8_t config_reg, uint16_t val, uint8_t offs, uint16_t mask)
{
    CHECK_ARG(dev);
    CHECK_ARG((config_reg == SGM58031_REG_CONFIG) || (config_reg == SGM58031_REG_CONFIG1));

    uint16_t old;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, config_reg, &old));
    I2C_DEV_CHECK(dev, write_reg(dev, config_reg, (old & ~(mask << offs)) | (val << offs)));

    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

#define READ_CONFIG(REG, OFFS, MASK, VAR)                                                                              \
    do                                                                                                                 \
    {                                                                                                                  \
        CHECK_ARG(VAR);                                                                                                \
        uint16_t bits;                                                                                                 \
        CHECK(read_conf_bits(dev, REG, OFFS, MASK, &bits));                                                            \
        *VAR = bits;                                                                                                   \
        return ESP_OK;                                                                                                 \
    }                                                                                                                  \
    while (0)

///////////////////////////////////////////////////////////////////////////////

esp_err_t sgm58031_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != SGM58031_ADDR_GND && addr != SGM58031_ADDR_VCC && addr != SGM58031_ADDR_SDA
        && addr != SGM58031_ADDR_SCL)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t sgm58031_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t sgm58031_is_busy(i2c_dev_t *dev, bool *busy)
{
    CHECK_ARG(dev && busy);

    uint16_t r;
    CHECK(read_conf_bits(dev, SGM58031_REG_CONFIG, OS_OFFSET, OS_MASK, &r));
    *busy = !r;

    return ESP_OK;
}

esp_err_t sgm58031_start_conversion(i2c_dev_t *dev)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG, 1, OS_OFFSET, OS_MASK);
}

esp_err_t sgm58031_get_value(i2c_dev_t *dev, int16_t *value)
{
    CHECK_ARG(dev && value);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, SGM58031_REG_CONVERSION, (uint16_t *)value));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t sgm58031_get_gain(i2c_dev_t *dev, sgm58031_gain_t *gain)
{
    READ_CONFIG(SGM58031_REG_CONFIG, PGA_OFFSET, PGA_MASK, gain);
}

esp_err_t sgm58031_set_gain(i2c_dev_t *dev, sgm58031_gain_t gain)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG, gain, PGA_OFFSET, PGA_MASK);
}

esp_err_t sgm58031_get_input_mux(i2c_dev_t *dev, sgm58031_mux_t *mux)
{
    READ_CONFIG(SGM58031_REG_CONFIG, MUX_OFFSET, MUX_MASK, mux);
}

esp_err_t sgm58031_set_input_mux(i2c_dev_t *dev, sgm58031_mux_t mux)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG, mux, MUX_OFFSET, MUX_MASK);
}

esp_err_t sgm58031_get_conv_mode(i2c_dev_t *dev, sgm58031_conv_mode_t *mode)
{
    READ_CONFIG(SGM58031_REG_CONFIG, MODE_OFFSET, MODE_MASK, mode);
}

esp_err_t sgm58031_set_conv_mode(i2c_dev_t *dev, sgm58031_conv_mode_t mode)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG, mode, MODE_OFFSET, MODE_MASK);
}

esp_err_t sgm58031_get_data_rate(i2c_dev_t *dev, sgm58031_data_rate_t *rate)
{
    CHECK_ARG(dev && rate);

    uint16_t bits;
    CHECK(read_conf_bits(dev, SGM58031_REG_CONFIG, DR_OFFSET, DR_MASK, &bits));

    *rate = bits;

    CHECK(read_conf_bits(dev, SGM58031_REG_CONFIG1, DR_SEL_OFFSET, DR_SEL_MASK, &bits));

    if (bits == 0x01) // DR SEL is high
    {
        *rate += 8;
    }

    return ESP_OK;
}

esp_err_t sgm58031_set_data_rate(i2c_dev_t *dev, sgm58031_data_rate_t rate)
{
    esp_err_t ret = ESP_FAIL;

    if (rate >= SGM58031_DATA_RATE_7_5)
    {
        ret = write_conf_bits(dev, SGM58031_REG_CONFIG1, 1, DR_SEL_OFFSET, DR_SEL_MASK);

        if (ret == ESP_OK)
        {
            ret = write_conf_bits(dev, SGM58031_REG_CONFIG, rate, DR_OFFSET, DR_MASK);
        }
    }
    else
    {
        ret = write_conf_bits(dev, SGM58031_REG_CONFIG1, 0, DR_SEL_OFFSET, DR_SEL_MASK);

        if (ret == ESP_OK)
        {
            ret = write_conf_bits(dev, SGM58031_REG_CONFIG, rate, DR_OFFSET, DR_MASK);
        }
    }

    return ret;
}

esp_err_t sgm58031_get_comp_mode(i2c_dev_t *dev, sgm58031_comp_mode_t *mode)
{
    READ_CONFIG(SGM58031_REG_CONFIG, COMP_MODE_OFFSET, COMP_MODE_MASK, mode);
}

esp_err_t sgm58031_set_comp_mode(i2c_dev_t *dev, sgm58031_comp_mode_t mode)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG, mode, COMP_MODE_OFFSET, COMP_MODE_MASK);
}

esp_err_t sgm58031_get_comp_polarity(i2c_dev_t *dev, sgm58031_comp_polarity_t *polarity)
{
    READ_CONFIG(SGM58031_REG_CONFIG, COMP_POL_OFFSET, COMP_POL_MASK, polarity);
}

esp_err_t sgm58031_set_comp_polarity(i2c_dev_t *dev, sgm58031_comp_polarity_t polarity)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG, polarity, COMP_POL_OFFSET, COMP_POL_MASK);
}

esp_err_t sgm58031_get_comp_latch(i2c_dev_t *dev, sgm58031_comp_latch_t *latch)
{
    READ_CONFIG(SGM58031_REG_CONFIG, COMP_LAT_OFFSET, COMP_LAT_MASK, latch);
}

esp_err_t sgm58031_set_comp_latch(i2c_dev_t *dev, sgm58031_comp_latch_t latch)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG, latch, COMP_LAT_OFFSET, COMP_LAT_MASK);
}

esp_err_t sgm58031_get_comp_queue(i2c_dev_t *dev, sgm58031_comp_queue_t *queue)
{
    READ_CONFIG(SGM58031_REG_CONFIG, COMP_QUE_OFFSET, COMP_QUE_MASK, queue);
}

esp_err_t sgm58031_set_comp_queue(i2c_dev_t *dev, sgm58031_comp_queue_t queue)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG, queue, COMP_QUE_OFFSET, COMP_QUE_MASK);
}

esp_err_t sgm58031_get_comp_low_thresh(i2c_dev_t *dev, int16_t *th)
{
    CHECK_ARG(dev && th);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, SGM58031_REG_THRESH_L, (uint16_t *)th));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t sgm58031_set_comp_low_thresh(i2c_dev_t *dev, int16_t th)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg(dev, SGM58031_REG_THRESH_L, th));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t sgm58031_get_comp_high_thresh(i2c_dev_t *dev, int16_t *th)
{
    CHECK_ARG(dev && th);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, SGM58031_REG_THRESH_H, (uint16_t *)th));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t sgm58031_set_comp_high_thresh(i2c_dev_t *dev, int16_t th)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg(dev, SGM58031_REG_THRESH_H, th));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t sgm58031_set_ain3_external_reference(i2c_dev_t *dev, bool enable)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG1, enable, EXT_REF_OFFSET, EXT_REF_MASK);
}

esp_err_t sgm58031_get_ain3_external_reference(i2c_dev_t *dev, bool *enable)
{
    READ_CONFIG(SGM58031_REG_CONFIG1, EXT_REF_OFFSET, EXT_REF_MASK, enable);
}

esp_err_t sgm58031_set_source_pair(i2c_dev_t *dev, bool enable)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG1, enable, BURNOUT_OFFSET, BURNOUT_MASK);
}

esp_err_t sgm58031_get_source_pair(i2c_dev_t *dev, bool *enable)
{
    READ_CONFIG(SGM58031_REG_CONFIG1, BURNOUT_OFFSET, BURNOUT_MASK, enable);
}

esp_err_t sgm58031_set_bus_leakage_circuit(i2c_dev_t *dev, bool enable)
{
    return write_conf_bits(dev, SGM58031_REG_CONFIG1, enable, BUS_FLEX_OFFSET, BUS_FLEX_MASK);
}

esp_err_t sgm58031_get_bus_leakage_circuit(i2c_dev_t *dev, bool *enable)
{
    READ_CONFIG(SGM58031_REG_CONFIG1, BUS_FLEX_OFFSET, BUS_FLEX_MASK, enable);
}

esp_err_t sgm58031_get_chip_id(i2c_dev_t *dev, uint8_t *id, uint8_t *version)
{
    CHECK_ARG(dev && id && version);

    I2C_DEV_TAKE_MUTEX(dev);
    uint16_t bits;
    I2C_DEV_CHECK(dev, read_reg(dev, SGM58031_REG_CHIP_ID, &bits));

    *version = (bits & 0xFF) >> 5;
    *id = (bits >> 8) & 0xFF;

    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t sgm58031_get_gn_trim1(i2c_dev_t *dev, uint16_t *trim_value)
{
    CHECK_ARG(dev && trim_value);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, SGM58031_REG_GN_TRIM1, trim_value));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t sgm58031_set_gn_trim1(i2c_dev_t *dev, uint16_t trim_value)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg(dev, SGM58031_REG_GN_TRIM1, trim_value));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

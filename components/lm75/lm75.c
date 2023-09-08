/*
 * Copyright (c) 2019 Tomoyuki Sakurai <y@trombik.org>
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
 * @file lm75.c
 *
 * ESP-IDF driver for LM75, a digital temperature sensor and thermal watchdog.
 *
 * The driver depends on i2cdev library in `esp-idf-lib`.
 *
 * The driver was written using LM75B.
 */
#include <esp_log.h>
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include "lm75.h"

#define I2C_FREQ_HZ 1000000 // 1 Mhz

#define LM75_MASK_SHUTDOWN      (1 << 0)
#define LM75_MASK_OS_COMP_INT   (1 << 1)
#define LM75_MASK_OS_POL        (1 << 2)
#define LM75_MASK_OS_F_QUE      ((1 << 4) | (1 << 3))

#define LM75_REG_CONF  0x01
#define LM75_REG_TEMP  0x00
#define LM75_REG_TOS   0x03
#define LM75_REG_THYST 0x02

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/* use CHECK_LOGE after I2C_DEV_TAKE_MUTEX(). */
#define CHECK_LOGE(dev, x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(dev); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

const static char *TAG = "lm75";

/* read_register* and write_register* must be protected with I2C_DEV_TAKE_MUTEX */
static esp_err_t read_register16(i2c_dev_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t data[] = { 0, 0 };
    CHECK(i2c_dev_read_reg(dev, reg, data, 2));
    *value = (data[0] << 8) | data[1];
    ESP_LOGV(TAG, "read_register16() reg: 0x%x value: 0x%x", reg, *value);
    return ESP_OK;
}

static esp_err_t read_register8(i2c_dev_t *dev, uint8_t reg, uint8_t *value)
{
    CHECK(i2c_dev_read_reg(dev, reg, value, 1));
    ESP_LOGV(TAG, "read_register8() reg: 0x%x value: 0x%x", reg, *value);
    return ESP_OK;
}

static esp_err_t write_register16(i2c_dev_t *dev, uint8_t reg, uint16_t value)
{
    ESP_LOGV(TAG, "write_register16(): reg: 0x%x, value: 0x%x", reg, value);
    return i2c_dev_write(dev, &reg, 2, &value, 2);
}

static esp_err_t write_register8(i2c_dev_t *dev, uint8_t reg, uint8_t value)
{
    ESP_LOGV(TAG, "write_register8(): reg: 0x%x, value: 0x%x", reg, value);
    return i2c_dev_write_reg(dev, reg, &value, 1);
}

esp_err_t lm75_read_temperature(i2c_dev_t *dev, float *value)
{
    CHECK_ARG(dev);
    uint16_t raw_data;

    I2C_DEV_TAKE_MUTEX(dev);
    CHECK_LOGE(dev, read_register16(dev, LM75_REG_TEMP, &raw_data),
            "lm75_read_temperature(): read_register16() failed: register: 0x%x", LM75_REG_TEMP);
    I2C_DEV_GIVE_MUTEX(dev);

    *value = (raw_data >> 5) * 0.125;
    return ESP_OK;
}

esp_err_t lm75_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr < LM75_I2C_ADDRESS_DEFAULT || addr > LM75_I2C_ADDRESS_MAX) {
        ESP_LOGE(TAG, "lm75_init_desc(): Invalid I2C address `0x%x`. address must not be less than 0x%x, not be more than 0x%x",
                addr, LM75_I2C_ADDRESS_DEFAULT, LM75_I2C_ADDRESS_MAX);
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

esp_err_t lm75_set_os_threshold(i2c_dev_t *dev, const float value)
{
    CHECK_ARG(dev);
    uint16_t reg_value;

    /*  two's complement format with the resolution of 0.5 C degree.
     *  7 LSB of the LSByte are equal to zero and should be ignored.
     */
    if (value < 0) {
        reg_value = ((uint16_t)(abs((int16_t)value) * 2) ^ 0xff) + 1;
    } else {
        reg_value = value * 2;
    }
    reg_value = reg_value << 7;
    /* when the value is 25.0f:
     * reg_value: 0x1900 9 bit reg_value: 0x32 value: 25.000000 */
    ESP_LOGV(TAG, "lm75_set_os_threshold(): reg_value: 0x%x 9 bit reg_value: 0x%x value: %f",
            reg_value, reg_value >> 7, value);

    I2C_DEV_TAKE_MUTEX(dev);
    CHECK_LOGE(dev, write_register16(dev, LM75_REG_TOS, reg_value),
            "lm75_set_os_threshold(): write_register16() failed: register 0x%x",
            LM75_REG_TOS);
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}


esp_err_t lm75_get_os_threshold(i2c_dev_t *dev, float *value)
{
    CHECK_ARG(dev);
    uint16_t reg_value;

    I2C_DEV_TAKE_MUTEX(dev);
    CHECK_LOGE(dev, read_register16(dev, LM75_REG_TOS, &reg_value),
            "lm75_get_os_threshold(): read_register16() failed: register: 0x%x", LM75_REG_TOS);
    I2C_DEV_GIVE_MUTEX(dev);

    ESP_LOGV(TAG, "lm75_get_os_threshold(): reg_value: 0x%x 9 bit reg_value: 0x%x", reg_value, reg_value >> 7);
    reg_value = reg_value >> 7;
    if (reg_value & (1 << 10)) {
        *value = ((reg_value | (1 << 10)) ^ 0xff) + 1;
        *value *= -1;
        *value /= 2;
    } else {
        *value = reg_value / 2;
    }

    return ESP_OK;
}

esp_err_t lm75_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

static
esp_err_t lm75_set_bits_register8(i2c_dev_t *dev, uint8_t reg, uint8_t mask)
{
    CHECK_ARG(dev);
    uint8_t value;
    ESP_LOGV(TAG, "lm75_set_bits_register8(): reg: 0x%x, mask: 0x%x", reg, mask);

    I2C_DEV_TAKE_MUTEX(dev);
    CHECK_LOGE(dev, read_register8(dev, reg, &value),
            "lm75_set_bits_register8(): read_register8() failed reg: 0x%x", reg);
    ESP_LOGV(TAG, "lm75_set_bits_register8(): value in register: 0x%x", value);
    if ((value & mask) != mask) {
        value |= mask;
        ESP_LOGV(TAG, "lm75_set_bits_register8(): updating register with value: 0x%x", value);
        CHECK_LOGE(dev, write_register8(dev, reg, value),
                "lm75_set_bits_register8(): write_register8() failed reg: 0x%x", reg);
    } else {
        ESP_LOGV(TAG, "lm75_set_bits_register8(): register unchanged");
    }
    I2C_DEV_GIVE_MUTEX(dev);
    return ESP_OK;

}

static
esp_err_t lm75_clear_bits_register8(i2c_dev_t *dev, uint8_t reg, uint8_t mask)
{
    CHECK_ARG(dev);
    uint8_t value;
    ESP_LOGV(TAG, "lm75_clear_bits_register8(): reg: 0x%x, mask: 0x%x", reg, mask);

    I2C_DEV_TAKE_MUTEX(dev);
    CHECK_LOGE(dev, read_register8(dev, reg, &value),
            "read_register8() failed: register: 0x%x", reg);
    if ((value & mask) == mask) {
        value ^= mask;
        ESP_LOGV(TAG, "lm75_clear_bits_register8(): updating register with value: 0x%x", value);
        CHECK_LOGE(dev, write_register8(dev, reg, value),
                "write_register8() failed: register 0x%x", reg);
    } else {
        ESP_LOGV(TAG, "lm75_clear_bits_register8(): register unchanged");
    }
    I2C_DEV_GIVE_MUTEX(dev);
    return ESP_OK;

}

esp_err_t lm75_shutdown(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    ESP_LOGV(TAG, "lm75_shutdown(): trying to set shutdown bit");

    return lm75_set_bits_register8(dev, LM75_REG_CONF, LM75_MASK_SHUTDOWN);
}

esp_err_t lm75_wakeup(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    ESP_LOGV(TAG, "lm75_wakeup(): trying to clear shutdown bit");

    return lm75_clear_bits_register8(dev, LM75_REG_CONF, LM75_MASK_SHUTDOWN);
}

esp_err_t lm75_set_os_polarity(i2c_dev_t *dev, const lm75_os_polarity_t v)
{
    ESP_LOGV(TAG, "lm75_set_os_polarity(): v: %d", v);
    if (v > 1) {
        ESP_LOGE(TAG, "lm75_set_os_polarity(): second argument must be %d or %d",
                LM75_OSP_LOW, LM75_OSP_HIGH);
        return ESP_ERR_INVALID_ARG;
    }

    if (v == LM75_OSP_HIGH) {
        return lm75_set_bits_register8(dev, LM75_REG_CONF, LM75_MASK_OS_POL);
    } else {
        return lm75_clear_bits_register8(dev, LM75_REG_CONF, LM75_MASK_OS_POL);
    }
}
esp_err_t lm75_get_os_polarity(i2c_dev_t *dev, uint8_t *v)
{
    CHECK_ARG(dev);
    uint8_t reg_value;

    I2C_DEV_TAKE_MUTEX(dev);
    CHECK_LOGE(dev, read_register8(dev, LM75_REG_CONF, &reg_value),
            "lm75_get_os_polarity(): read_register8() failed: reg: 0x%x", LM75_REG_CONF);
    I2C_DEV_GIVE_MUTEX(dev);

    *v = (reg_value & LM75_MASK_OS_POL) == 0 ? 0 : 1;
    return ESP_OK;
}

esp_err_t lm75_set_os_mode(i2c_dev_t *dev, lm75_os_mode_t v)
{
    ESP_LOGV(TAG, "lm75_set_os_mode(): v: %d", v);
    if (v > 1) {
        ESP_LOGE(TAG, "lm75_set_os_mode(): second argument must be %d or %d",
                LM75_OS_MODE_COMP, LM75_OS_MODE_INT);
        return ESP_ERR_INVALID_ARG;
    }

    if (v == LM75_OS_MODE_INT) {
        return lm75_set_bits_register8(dev, LM75_REG_CONF, LM75_MASK_OS_COMP_INT);
    } else {
        return lm75_clear_bits_register8(dev, LM75_REG_CONF, LM75_MASK_OS_COMP_INT);
    }
}
/*
 * Configuration register (0x00)
 * | [7:5]    |   4    |  3   |   2    |     1       |    0     |
 * | Reserved | OS_F_QUE[1:0] | PS_POL | OS_COMP_INT | SHUTDOWN |
 */
esp_err_t lm75_init(i2c_dev_t *dev, const lm75_config_t config)
{
    CHECK_ARG(dev);
    uint8_t value = 0;

    value = (config.mode           << 0) |
            (config.os_mode        << 1) |
            (config.os_mode        << 2) |
            (config.os_fault_queue << 3);

    I2C_DEV_TAKE_MUTEX(dev);
    CHECK_LOGE(dev, write_register8(dev, LM75_REG_CONF, value),
            "lm75_init(): write_register8() failed");
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

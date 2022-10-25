/*
 * Copyright (c) 2022 Tomoyuki Sakurai <y@trombik.org>
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

#if !defined(__DPS310__HELPER_I2C__H__)
#define __DPS310__HELPER_I2C__H__

/* standard headers */
#include <inttypes.h>

/* esp-idf headers */
#include <esp_err.h>
#include <esp_log.h>
#include <i2cdev.h>

/* private headers */
#include "helper_macro.h"
#include "helper_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

static const char *TAG = "dps310/helper_i2c";

/* count the number of trailing zero in a value, usually bitmasks. */
static uint8_t count_trailing_zero_bits(uint8_t v)
{
    uint8_t count;
    if (v)
    {
        v = (v ^ (v - 1)) >> 1;
        for (count = 0; v; count++)
        {
            v >>= 1;
        }
    }
    else
    {
        count = 8;
    }
    return count;
}

esp_err_t _read_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(dev, reg, val, 1);
}

esp_err_t _read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, _read_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);
    return ESP_OK;
}

esp_err_t _read_reg_mask(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t *val)
{
    uint8_t reg_value = 0;

    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, _read_reg_nolock(dev, reg, &reg_value));
    I2C_DEV_GIVE_MUTEX(dev);
    *val = (reg_value & mask) >> count_trailing_zero_bits(mask);
    ESP_LOGV(TAG, "reg_value: %02X, val: %02X", reg_value, *val);
    return ESP_OK;
}

esp_err_t _write_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(dev, reg, &val, 1);
}

esp_err_t _update_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t reg_value = 0;
    uint8_t n_shift = 0;

    CHECK_ARG(dev);
    CHECK(_read_reg_nolock(dev, reg, &reg_value));

    n_shift = count_trailing_zero_bits(mask);
    if (((reg_value >> n_shift) & val) == val)
    {
        ESP_LOGD(TAG, "register unchanged");
    }
    else
    {
        reg_value = (reg_value & (~mask)) | (val << n_shift);
        CHECK(_write_reg_nolock(dev, reg, reg_value));
    }
    return ESP_OK;
}

esp_err_t _write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *value)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(dev);
    err = _write_reg_nolock(dev, reg, *value);
    I2C_DEV_GIVE_MUTEX(dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_write_reg(): %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t _update_reg(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, _update_reg_nolock(dev, reg, mask, val));
    I2C_DEV_GIVE_MUTEX(dev);
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif

#endif // __DPS310__HELPER_I2C__H__
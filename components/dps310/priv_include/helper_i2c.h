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
#include <i2cdev.h>

/* private headers */
#include "helper_macro.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t _read_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t *val);

esp_err_t _read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val);

esp_err_t _read_reg_mask(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t *val);

esp_err_t _write_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t val);

esp_err_t _write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *value);

esp_err_t _update_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val);

esp_err_t _update_reg(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val);

esp_err_t _wait_for_reg_bits(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val, uint8_t max_attempt, uint16_t delay);

#ifdef __cplusplus
}
#endif

#endif // __DPS310__HELPER_I2C__H__

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

/*
 * A private header for I2C interface. Most of them are a wrapper of i2cdev
 * functions.
 *
 * Functions in the header should be prefixed with `_` to indicate they are
 * a private function.
 *
 * A postfix, `_nolock` in function name means the function does not acquire
 * I2C lock in `i2cdev`.
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

/**
 * @brief Read a single byte from a 8-bit resister with locking.
 */
esp_err_t _read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val);

/**
 * @brief Read a masked value from a 8-bit resister with locking.
 *
 * The returned value is bit-shifted. When `mask` is `0b1111000`, and the
 * value in the resister is `0b00010000`, `val` is `0b0001`.
 */
esp_err_t _read_reg_mask(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t *val);

/**
 * @brief Write a single byte to a 8-bit resister with locking.
 */
esp_err_t _write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *value);

/**
 * @brief Update a 8-bit resister with a masked value without locking.
 *
 * `mask` is the mask to update, and `val` is the value. The function reads
 * the resister, and see if the bit values in the resister needs update. If
 * the bit value is identical with `val`, it does not update the resister.
 *
 * `val` is automatically bit-shifted when updating the resister.
 *
 * Useful to update specific bits in a resister.
 *
 * See also: _update_reg().
 */
esp_err_t _update_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val);

/**
 * @brief Update a 8-bit resister with a masked value.
 */
esp_err_t _update_reg(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val);

esp_err_t _wait_for_reg_bits(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val, uint8_t max_attempt, uint16_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif // __DPS310__HELPER_I2C__H__

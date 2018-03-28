/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ruslan V. Uss (https://github.com/UncleRus)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef __I2C_UTILS_H__
#define __I2C_UTILS_H__

#include <driver/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    i2c_port_t port;
    uint8_t addr;
} i2c_dev_t;

esp_err_t i2c_setup_master(i2c_port_t i2c_num, gpio_num_t scl_pin, gpio_num_t sda_pin, uint32_t clk_freq);

esp_err_t i2c_read_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, void *res, size_t size);

esp_err_t i2c_write_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, void *data, size_t size);


inline esp_err_t i2c_dev_read_register(i2c_dev_t *dev, uint8_t reg, void *res, size_t size)
{
    return i2c_read_register(dev->port, dev->addr, reg, res, size);
}

inline esp_err_t i2c_dev_write_register(i2c_dev_t *dev, uint8_t reg, void *data, size_t size)
{
    return i2c_write_register(dev->port, dev->addr, reg, data, size);
}

#ifdef __cplusplus
}
#endif

#endif /* __I2C_UTILS_H__ */

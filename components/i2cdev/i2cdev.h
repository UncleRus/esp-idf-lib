/**
 * @file i2cdev.h
 *
 * ESP-32 I2C master thread-safe functions for communication with I2C slave
 *
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
#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <driver/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * I2C device descriptor
 */
typedef struct
{
    i2c_port_t port;
    i2c_config_t cfg;
    uint8_t addr;       //!< Unshifted address
} i2c_dev_t;

/**
 * @brief Init I2Cdev lib
 * The function must be called before any other
 * functions of this library
 * @return ESP_OK on success
 */
esp_err_t i2cdev_init();

/**
 * @brief Finish work with I2CDev lib
 * @return ESP_OK on success
 */
esp_err_t i2cdev_done();

/**
 * @brief Read from slave device
 * Issue a send operation of \p out_data register adress, followed by reading \p in_size bytes
 * from slave into \p in_data .
 * Function is thread-safe.
 * @param[in] dev Device descriptor
 * @param[in] out_data Pointer to data to send if non-null
 * @param[in] out_size Size of data to send
 * @param[out] in_data Pointer to input data buffer
 * @param[in] in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data,
        size_t out_size, void *in_data, size_t in_size);

/**
 * @brief Write to slave device
 * Write \p out_size bytes from \p out_data to slave into \p out_reg register address.
 * Function is thread-safe.
 * @param[in] dev Device descriptor
 * @param[in] out_reg Pointer to register address to send if non-null
 * @param[in] out_reg_size Size of register address
 * @param[in] out_data Pointer to data to send
 * @param[in] out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg,
        size_t out_reg_size, const void *out_data, size_t out_size);

/**
 * @brief Read from register with an 8-bit address
 * Shortcut to i2c_dev_read().
 * @param[in] dev Device descriptor
 * @param[in] reg Register address
 * @param[out] in_data Pointer to input data buffer
 * @param[in] in_size Number of byte to read
 * @return ESP_OK on success
 */
inline esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg,
        void *in_data, size_t in_size)
{
    return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}

/**
 * @brief Write to register with an 8-bit address
 * Shortcut to i2c_dev_write().
 * @param[in] dev Device descriptor
 * @param[in] reg Register address
 * @param[in] out_data Pointer to data to send
 * @param[in] out_size Size of data to send
 * @return ESP_OK on success
 */
inline esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg,
        const void *out_data, size_t out_size)
{
    return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}

#ifdef __cplusplus
}
#endif

#endif /* __I2CDEV_H__ */

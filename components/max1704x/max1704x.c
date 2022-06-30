/*
 * Copyright (c) 2022 Joshua Butler, MD, MHI <josh.butler929@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file max1704x.c
 * @defgroup max1704x max1704x
 * @{
 *
 * ESP-IDF driver for MAX1704x battery fuel gauge
 * 
 * Copyright (c) 2022 Joshua Butler, MD, MHI <josh.butler929@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include <esp_log.h>

#include "max1704x.h"

#define I2C_FREQ_HZ 400000

/**
 * MAX1704X registers
 */
#define MAX1704X_REGISTER_VCELL         0x02
#define MAX1704X_REGISTER_SOC           0x04
#define MAX1704X_REGISTER_MODE          0x06
#define MAX1704X_REGISTER_VERSION       0x08
#define MAX1704X_REGISTER_CONFIG        0x0C
#define MAX1704X_REGISTER_CRATE         0x16
#define MAX1704X_REGISTER_COMMAND       0xFE
#define MAX1704X_RESET_COMMAND          0x5400
#define MAX1704X_QUICKSTART_MODE        0x4000

/**
 * MAX1704X modes
 */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define MAX1704X_DEFER_ADDRESS          (uint8_t)0

/**
 * MAX1704X constants
 */
#define MAX17043_MV_PRECISION           1.25
#define MAX17048_MV_PRECISION           0.078125
#define MAX1704X_CRATE_PRECISION        0.208

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static char *tag = "max1704x";

/**
 * Private functions
 */

int16_t be16_to_cpu_signed(const uint8_t data[2]) {
    int16_t r;
    uint16_t u = (unsigned)data[1] | ((unsigned)data[0] << 8);
    memcpy(&r, &u, sizeof r);
    return r;
}

int16_t le16_to_cpu_signed(const uint8_t data[2]) {
    int16_t r;
    uint16_t u = (unsigned)data[0] | ((unsigned)data[1] << 8);
    memcpy(&r, &u, sizeof r);
    return r;
}

/**
 * Public functions
 */

esp_err_t max1704x_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = MAX1704X_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t max1704x_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(dev);
}

esp_err_t max1704x_init(max1704x_t *dev, i2c_dev_t *i2c_dev, max1704x_model_t model)
{
    CHECK_ARG(dev);
    CHECK_ARG(i2c_dev);
    CHECK_ARG(model == MAX17043_4 || model == MAX17048_9);
    dev->i2c_dev = i2c_dev;
    dev->model = model;

    return ESP_OK;
}

esp_err_t max1704x_quickstart(max1704x_t *dev)
{
    uint8_t data[2];
    
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(dev->i2c_dev);

    data[0] = 0x40;
    data[1] = 0x00;

    I2C_DEV_CHECK(dev->i2c_dev, i2c_dev_write_reg(dev->i2c_dev, MAX1704X_REGISTER_MODE, data, 2));
    I2C_DEV_GIVE_MUTEX(dev->i2c_dev);
    ESP_LOGI(tag, "MAX1704X Quickstart");

    return ESP_OK;
}

esp_err_t max1704x_get_voltage(max1704x_t *dev, float *voltage)
{
    *voltage = 0;
    uint8_t data[2];
    int value;

    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(dev->i2c_dev);
    I2C_DEV_CHECK(dev->i2c_dev, i2c_dev_read_reg(dev->i2c_dev, MAX1704X_REGISTER_VCELL, data, 2));
    I2C_DEV_GIVE_MUTEX(dev->i2c_dev);
    // ESP_LOG_BUFFER_HEXDUMP("voltage", data, 2, ESP_LOG_INFO);

    if (dev->model == MAX17043_4) {
        value = (data[0] << 4) | (data[1] >> 4);
        *voltage = (value * MAX17043_MV_PRECISION) / 1000;
    } else {
        *voltage = (((float)data[0] * 256 + (float)data[1]) * MAX17048_MV_PRECISION) / 1000;
    }
    return ESP_OK;
}

esp_err_t max1704x_get_soc(max1704x_t *dev, float *soc)
{
    *soc = 0;
    uint8_t data[2];
    
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(dev->i2c_dev);
    I2C_DEV_CHECK(dev->i2c_dev, i2c_dev_read_reg(dev->i2c_dev, MAX1704X_REGISTER_SOC, data, 2));
    I2C_DEV_GIVE_MUTEX(dev->i2c_dev);
    // ESP_LOG_BUFFER_HEXDUMP("soc", data, 2, ESP_LOG_INFO);
    
   *soc = (float)data[0];
    *soc += ((float)data[1])/256;
    if (*soc > 100) {
        *soc = 100.0;
    }
    return ESP_OK;
}

esp_err_t max1704x_get_crate(max1704x_t *dev, float *crate)
{
    *crate = 0;
    int16_t crate_value;
    uint8_t data[2];
    
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(dev->i2c_dev);
    I2C_DEV_CHECK(dev->i2c_dev, i2c_dev_read_reg(dev->i2c_dev, MAX1704X_REGISTER_CRATE, data, 2));
    I2C_DEV_GIVE_MUTEX(dev->i2c_dev);
    // ESP_LOG_BUFFER_HEXDUMP("crate", data, 2, ESP_LOG_INFO);
    
    if (dev->model == MAX17043_4) {
        ESP_LOGW(tag, "MAX1704X_REGISTER_CRATE is not supported by MAX17043");
    } else {
        crate_value = be16_to_cpu_signed(data);
        ESP_LOGI(tag, "crateValue: %d", (int)crate_value);
        *crate = ((float)crate_value * MAX1704X_CRATE_PRECISION);
    }
    return ESP_OK;
}

esp_err_t  max1704x_get_version(max1704x_t *dev, uint16_t *version)
{
    CHECK_ARG(dev);
    CHECK_ARG(version);
    uint8_t data[2];
    
    I2C_DEV_TAKE_MUTEX(dev->i2c_dev);
    I2C_DEV_CHECK(dev->i2c_dev, i2c_dev_read_reg(dev->i2c_dev, MAX1704X_REGISTER_VERSION, data, 2));
    I2C_DEV_GIVE_MUTEX(dev->i2c_dev);
    // ESP_LOG_BUFFER_HEXDUMP("version", data, 2, ESP_LOG_INFO);
    
    *version = (data[0] << 8) | data[1];
    return ESP_OK;
}

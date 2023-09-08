/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file mcp342x.c
 *
 * ESP-IDF driver for 18-Bit, multi-channel delta-sigma
 * ADC MCP3426/MCP3427/MCP3428
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "mcp342x.h"

#define I2C_FREQ_HZ 400000 // 400kHz

static const char *TAG = "mcp342x";

#define BV(x) (1 << (x))

#define BIT_RDY   BV(7)
#define BIT_MODE  BV(4)

#define POS_CHAN 5
#define POS_SR   2
#define POS_GAIN 0

#define MASK_VAL 3

#define SIGN12 0xfffff000
#define SIGN14 0xffffc000
#define SIGN16 0xffff0000
#define SIGN18 0xfffc0000

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const uint32_t sample_time[] = {
    [MCP342X_RES_12] = 4167,
    [MCP342X_RES_14] = 16667,
    [MCP342X_RES_16] = 66667,
    [MCP342X_RES_18] = 266667
};

static const float lsb[] = {
    [MCP342X_RES_12] = 0.001,
    [MCP342X_RES_14] = 0.00025,
    [MCP342X_RES_16] = 0.0000625,
    [MCP342X_RES_18] = 0.000015625
};

static const int gain_val[] = {
    [MCP342X_GAIN1] = 1,
    [MCP342X_GAIN2] = 2,
    [MCP342X_GAIN4] = 4,
    [MCP342X_GAIN8] = 8
};

static void get_cfg(mcp342x_t *dev, uint8_t reg)
{
    dev->mode = (reg & BIT_MODE) ? MCP342X_CONTINUOUS : MCP342X_ONESHOT;
    dev->channel =    (reg >> POS_CHAN) & MASK_VAL;
    dev->resolution = (reg >> POS_SR)   & MASK_VAL;
    dev->gain =       (reg >> POS_GAIN) & MASK_VAL;
}

static inline uint8_t get_reg(mcp342x_t *dev)
{
    return (dev->mode == MCP342X_CONTINUOUS ? BIT_MODE : 0)
        | ((dev->channel & MASK_VAL) << POS_CHAN)
        | ((dev->resolution & MASK_VAL) << POS_SR)
        | ((dev->gain & MASK_VAL) << POS_GAIN);
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t mcp342x_init_desc(mcp342x_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev && addr >= MCP342X_ADDR_MIN && addr <= MCP342X_ADDR_MAX);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t mcp342x_free_desc(mcp342x_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t mcp342x_get_data(mcp342x_t *dev, int32_t *data, bool *ready)
{
    CHECK_ARG(dev);

    uint8_t buf[4] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, buf, sizeof(buf)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    uint8_t reg = buf[(buf[3] & (MCP342X_RES_18 << POS_SR)) == (MCP342X_RES_18 << POS_SR) ? 3 : 2];
    get_cfg(dev, reg);
    if (ready) *ready = !(reg & BIT_RDY);

    if (!data) return ESP_OK;

    uint32_t r = 0;
    switch (dev->resolution)
    {
        case MCP342X_RES_12:
            r = (buf[0] << 8) | buf[1];
            if (r & BV(11))
                r |= SIGN12;
            break;
        case MCP342X_RES_14:
            r = (buf[0] << 8) | buf[1];
            if (r & BV(13))
                r |= SIGN14;
            break;
        case MCP342X_RES_16:
            r = (buf[0] << 8) | buf[1];
            if (r & BV(15))
                r |= SIGN16;
            break;
        case MCP342X_RES_18:
            r = (buf[0] << 16) | (buf[1] << 8) | buf[2];
            if (r & BV(17))
                r |= SIGN18;
            break;
    }
    *data = *((int32_t *)&r);

    return ESP_OK;
}

esp_err_t mcp342x_get_voltage(mcp342x_t *dev, float *volts, bool *ready)
{
    CHECK_ARG(volts);

    int32_t raw;
    CHECK(mcp342x_get_data(dev, &raw, ready));
    *volts = lsb[dev->resolution] * raw / gain_val[dev->gain];

    return ESP_OK;
}

esp_err_t mcp342x_get_sample_time_us(mcp342x_t *dev, uint32_t *us)
{
    CHECK_ARG(dev && us && dev->resolution <= MCP342X_RES_18);

    *us = sample_time[dev->resolution];
    return ESP_OK;
}

esp_err_t mcp342x_set_config(mcp342x_t *dev)
{
    CHECK_ARG(dev);

    uint8_t r = get_reg(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, NULL, 0, &r, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mcp342x_get_config(mcp342x_t *dev)
{
    return mcp342x_get_data(dev, NULL, NULL);
}

esp_err_t mcp342x_start_conversion(mcp342x_t *dev)
{
    CHECK_ARG(dev);

    uint8_t r = get_reg(dev) | BIT_RDY;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, NULL, 0, &r, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mcp342x_oneshot_conversion(mcp342x_t *dev, int32_t *data)
{
    CHECK_ARG(dev && data);

    dev->mode = MCP342X_ONESHOT;

    uint32_t st;
    CHECK(mcp342x_get_sample_time_us(dev, &st));
    CHECK(mcp342x_start_conversion(dev));
    vTaskDelay(pdMS_TO_TICKS(st / 1000 + 1));
    bool ready;
    CHECK(mcp342x_get_data(dev, data, &ready));
    if (!ready)
    {
        ESP_LOGE(TAG, "Data not ready");
        return ESP_FAIL;
    }

    return ESP_OK;
}

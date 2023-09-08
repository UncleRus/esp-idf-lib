/*
 * Copyright (c) 2016 Brian Schwind <https://github.com/bschwind>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file tsl2561.c
 *
 * ESP-IDF driver for TSL2561 light-to-digital converter
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Brian Schwind <https://github.com/bschwind>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "tsl2561.h"

#define I2C_FREQ_HZ 400000 // 400kHz

static const char *TAG = "tsl2561";

// Registers
#define TSL2561_REG_COMMAND          0x80
#define TSL2561_REG_CONTROL          0x00
#define TSL2561_REG_TIMING           0x01
#define TSL2561_REG_THRESHOLD_LOW_0  0x02
#define TSL2561_REG_THRESHOLD_LOW_1  0x03
#define TSL2561_REG_THRESHOLD_HIGH_0 0x04
#define TSL2561_REG_THRESHOLD_HIGH_1 0x05
#define TSL2561_REG_INTERRUPT        0x06
#define TSL2561_REG_PART_ID          0x0A
#define TSL2561_REG_CHANNEL_0_LOW    0x0C
#define TSL2561_REG_CHANNEL_0_HIGH   0x0D
#define TSL2561_REG_CHANNEL_1_LOW    0x0E
#define TSL2561_REG_CHANNEL_1_HIGH   0x0F

// TSL2561 Misc Values
#define TSL2561_ON        0x03
#define TSL2561_OFF       0x00
#define TSL2561_READ_WORD 0x20

// Integration times in useconds
#define TSL2561_INTEGRATION_TIME_13MS  20
#define TSL2561_INTEGRATION_TIME_101MS 110
#define TSL2561_INTEGRATION_TIME_402MS 420 // Default

// Calculation constants
#define LUX_SCALE     14
#define RATIO_SCALE   9
#define CH_SCALE      10
#define CHSCALE_TINT0 0x7517
#define CHSCALE_TINT1 0x0fe7

// Constants from the TSL2561 data sheet
#define K1T 0x0040 // 0.125 * 2^RATIO_SCALE
#define B1T 0x01f2 // 0.0304 * 2^LUX_SCALE
#define M1T 0x01be // 0.0272 * 2^LUX_SCALE
#define K2T 0x0080 // 0.250 * 2^RATIO_SCALE
#define B2T 0x0214 // 0.0325 * 2^LUX_SCALE
#define M2T 0x02d1 // 0.0440 * 2^LUX_SCALE
#define K3T 0x00c0 // 0.375 * 2^RATIO_SCALE
#define B3T 0x023f // 0.0351 * 2^LUX_SCALE
#define M3T 0x037b // 0.0544 * 2^LUX_SCALE
#define K4T 0x0100 // 0.50 * 2^RATIO_SCALE
#define B4T 0x0270 // 0.0381 * 2^LUX_SCALE
#define M4T 0x03fe // 0.0624 * 2^LUX_SCALE
#define K5T 0x0138 // 0.61 * 2^RATIO_SCALE
#define B5T 0x016f // 0.0224 * 2^LUX_SCALE
#define M5T 0x01fc // 0.0310 * 2^LUX_SCALE
#define K6T 0x019a // 0.80 * 2^RATIO_SCALE
#define B6T 0x00d2 // 0.0128 * 2^LUX_SCALE
#define M6T 0x00fb // 0.0153 * 2^LUX_SCALE
#define K7T 0x029a // 1.3 * 2^RATIO_SCALE
#define B7T 0x0018 // 0.00146 * 2^LUX_SCALE
#define M7T 0x0012 // 0.00112 * 2^LUX_SCALE
#define K8T 0x029a // 1.3 * 2^RATIO_SCALE
#define B8T 0x0000 // 0.000 * 2^LUX_SCALE
#define M8T 0x0000 // 0.000 * 2^LUX_SCALE
#define K1C 0x0043 // 0.130 * 2^RATIO_SCALE
#define B1C 0x0204 // 0.0315 * 2^LUX_SCALE
#define M1C 0x01ad // 0.0262 * 2^LUX_SCALE
#define K2C 0x0085 // 0.260 * 2^RATIO_SCALE
#define B2C 0x0228 // 0.0337 * 2^LUX_SCALE
#define M2C 0x02c1 // 0.0430 * 2^LUX_SCALE
#define K3C 0x00c8 // 0.390 * 2^RATIO_SCALE
#define B3C 0x0253 // 0.0363 * 2^LUX_SCALE
#define M3C 0x0363 // 0.0529 * 2^LUX_SCALE
#define K4C 0x010a // 0.520 * 2^RATIO_SCALE
#define B4C 0x0282 // 0.0392 * 2^LUX_SCALE
#define M4C 0x03df // 0.0605 * 2^LUX_SCALE
#define K5C 0x014d // 0.65 * 2^RATIO_SCALE
#define B5C 0x0177 // 0.0229 * 2^LUX_SCALE
#define M5C 0x01dd // 0.0291 * 2^LUX_SCALE
#define K6C 0x019a // 0.80 * 2^RATIO_SCALE
#define B6C 0x0101 // 0.0157 * 2^LUX_SCALE
#define M6C 0x0127 // 0.0180 * 2^LUX_SCALE
#define K7C 0x029a // 1.3 * 2^RATIO_SCALE
#define B7C 0x0037 // 0.00338 * 2^LUX_SCALE
#define M7C 0x002b // 0.00260 * 2^LUX_SCALE
#define K8C 0x029a // 1.3 * 2^RATIO_SCALE
#define B8C 0x0000 // 0.000 * 2^LUX_SCALE
#define M8C 0x0000 // 0.000 * 2^LUX_SCALE

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

static inline esp_err_t write_register(tsl2561_t *dev, uint8_t reg, uint8_t value)
{
    return i2c_dev_write_reg(&dev->i2c_dev, TSL2561_REG_COMMAND | reg, &value, 1);
}

static inline esp_err_t read_register(tsl2561_t *dev, uint8_t reg, uint8_t *value)
{
    return i2c_dev_read_reg(&dev->i2c_dev, TSL2561_REG_COMMAND | reg, value, 1);
}

static inline esp_err_t read_register_16(tsl2561_t *dev, uint8_t low_register_addr, uint16_t *value)
{
    uint8_t buf[2];

    CHECK(i2c_dev_read_reg(&dev->i2c_dev, TSL2561_REG_COMMAND | TSL2561_READ_WORD | low_register_addr, buf, 2));
    *value = ((uint16_t)buf[1] << 8) | buf[0];

    return ESP_OK;
}

static inline esp_err_t enable(tsl2561_t *dev)
{
    return write_register(dev, TSL2561_REG_CONTROL, TSL2561_ON);
}

static inline esp_err_t disable(tsl2561_t *dev)
{
    return write_register(dev, TSL2561_REG_CONTROL, TSL2561_OFF);
}

static inline esp_err_t get_channel_data(tsl2561_t *dev, uint16_t *channel0, uint16_t *channel1)
{
    int sleep_ms;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, enable(dev));

    // Since we just enabled the chip, we need to sleep
    // for the chip's integration time so it can gather a reading
    switch (dev->integration_time)
    {
        case TSL2561_INTEGRATION_13MS:
            sleep_ms = TSL2561_INTEGRATION_TIME_13MS;
            break;
        case TSL2561_INTEGRATION_101MS:
            sleep_ms = TSL2561_INTEGRATION_TIME_101MS;
            break;
        default:
            sleep_ms = TSL2561_INTEGRATION_TIME_402MS;
            break;
    }
    SLEEP_MS(sleep_ms);

    I2C_DEV_CHECK(&dev->i2c_dev, read_register_16(dev, TSL2561_REG_CHANNEL_0_LOW, channel0));
    I2C_DEV_CHECK(&dev->i2c_dev, read_register_16(dev, TSL2561_REG_CHANNEL_1_LOW, channel1));

    I2C_DEV_CHECK(&dev->i2c_dev, disable(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    ESP_LOGD(TAG, "integration time: %d ms channel0: 0x%x channel1: 0x%x", sleep_ms, *channel0, *channel1);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t tsl2561_init_desc(tsl2561_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != TSL2561_I2C_ADDR_GND && addr != TSL2561_I2C_ADDR_FLOAT && addr != TSL2561_I2C_ADDR_VCC)
    {
        ESP_LOGE(TAG, "Invalid I2C address `0x%x`: must be one of 0x%x, 0x%x, 0x%x",
                addr, TSL2561_I2C_ADDR_GND, TSL2561_I2C_ADDR_FLOAT, TSL2561_I2C_ADDR_VCC);
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t tsl2561_free_desc(tsl2561_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t tsl2561_init(tsl2561_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, enable(dev));
    uint8_t control_reg;
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL2561_REG_CONTROL, &control_reg));
    if ((control_reg & TSL2561_ON)!= TSL2561_ON)
    {
        ESP_LOGE(TAG, "Error initializing tsl2561, control register wasn't set to ON");
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Fetch the package type
    uint8_t part_reg;
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL2561_REG_PART_ID, &part_reg));
    dev->package_type = part_reg >> 6;

    // Fetch the gain and integration time
    uint8_t timing_reg;
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL2561_REG_TIMING, &timing_reg));
    dev->gain = timing_reg & 0x10;
    dev->integration_time = timing_reg & 0x03;

    I2C_DEV_CHECK(&dev->i2c_dev, disable(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2561_set_integration_time(tsl2561_t *dev, tsl2561_integration_time_t integration_time)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, enable(dev));

    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2561_REG_TIMING, integration_time | dev->gain));
    dev->integration_time = integration_time;

    I2C_DEV_CHECK(&dev->i2c_dev, disable(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2561_set_gain(tsl2561_t *dev, tsl2561_gain_t gain)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, enable(dev));

    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2561_REG_TIMING, gain | dev->integration_time));
    dev->gain = gain;

    I2C_DEV_CHECK(&dev->i2c_dev, disable(dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2561_read_lux(tsl2561_t *dev, uint32_t *lux)
{
    CHECK_ARG(dev && lux);

    uint32_t ch_scale, channel1, channel0;

    switch (dev->integration_time)
    {
        case TSL2561_INTEGRATION_13MS:
            ch_scale = CHSCALE_TINT0;
            break;
        case TSL2561_INTEGRATION_101MS:
            ch_scale = CHSCALE_TINT1;
            break;
        default:
            ch_scale = (1 << CH_SCALE);
            break;
    }

    // Scale if gain is 1x
    if (dev->gain == TSL2561_GAIN_1X)
        // 16x is nominal, so if the gain is set to 1x then
        // we need to scale by 16
        ch_scale = ch_scale << 4;

    uint16_t ch0 = 0;
    uint16_t ch1 = 0;

    CHECK(get_channel_data(dev, &ch0, &ch1));

    // Scale the channel values
    channel0 = (ch0 * ch_scale) >> CH_SCALE;
    channel1 = (ch1 * ch_scale) >> CH_SCALE;

    // Find the ratio of the channel values (channel1 / channel0)
    // Protect against divide by zero
    uint32_t ratio1 = 0;
    if (channel0 != 0)
        ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;

    // Round the ratio value
    uint32_t ratio = (ratio1 + 1) >> 1;

    uint32_t b = 0;
    uint32_t m = 0;
    switch (dev->package_type)
    {
        case TSL2561_PACKAGE_CS:
            if (ratio <= K1C)
            {
                b = B1C;
                m = M1C;
            }
            else if (ratio <= K2C)
            {
                b = B2C;
                m = M2C;
            }
            else if (ratio <= K3C)
            {
                b = B3C;
                m = M3C;
            }
            else if (ratio <= K4C)
            {
                b = B4C;
                m = M4C;
            }
            else if (ratio <= K5C)
            {
                b = B5C;
                m = M5C;
            }
            else if (ratio <= K6C)
            {
                b = B6C;
                m = M6C;
            }
            else if (ratio <= K7C)
            {
                b = B7C;
                m = M7C;
            }
            else if (ratio > K8C)
            {
                b = B8C;
                m = M8C;
            }

            break;
        case TSL2561_PACKAGE_T_FN_CL:
            if (ratio <= K1T)
            {
                b = B1T;
                m = M1T;
            }
            else if (ratio <= K2T)
            {
                b = B2T;
                m = M2T;
            }
            else if (ratio <= K3T)
            {
                b = B3T;
                m = M3T;
            }
            else if (ratio <= K4T)
            {
                b = B4T;
                m = M4T;
            }
            else if (ratio <= K5T)
            {
                b = B5T;
                m = M5T;
            }
            else if (ratio <= K6T)
            {
                b = B6T;
                m = M6T;
            }
            else if (ratio <= K7T)
            {
                b = B7T;
                m = M7T;
            }
            else if (ratio > K8T)
            {
                b = B8T;
                m = M8T;
            }

            break;
        default:
            ESP_LOGE(TAG, "Invalid package type in CalculateLux");
            return ESP_ERR_NOT_SUPPORTED;
    }

    int32_t temp;
    temp = ((channel0 * b) - (channel1 * m));

    // Round lsb (2^(LUX_SCALE−1))
    temp += (1 << (LUX_SCALE - 1));

    // Strip off fractional portion
    *lux = temp >> LUX_SCALE;

    return ESP_OK;
}

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Frank Bargstedt
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file bmp180.c
 *
 * ESP-IDF driver for BMP180 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2015 Frank Bargstedt\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "bmp180.h"
#include <inttypes.h>
#include <esp_err.h>
#include <esp_log.h>
#include <ets_sys.h>
#include <esp_idf_lib_helpers.h>

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf

static const char *TAG = "bmp180";

#define BMP180_RX_QUEUE_SIZE      10
#define BMP180_TASK_PRIORITY      9

#define BMP180_VERSION_REG        0xD0
#define BMP180_CONTROL_REG        0xF4
#define BMP180_RESET_REG          0xE0
#define BMP180_OUT_MSB_REG        0xF6
#define BMP180_OUT_LSB_REG        0xF7
#define BMP180_OUT_XLSB_REG       0xF8

#define BMP180_CALIBRATION_REG    0xAA

// Values for BMP180_CONTROL_REG
#define BMP180_MEASURE_TEMP       0x2E
#define BMP180_MEASURE_PRESS      0x34

// CHIP ID stored in BMP180_VERSION_REG
#define BMP180_CHIP_ID            0x55

// Reset value for BMP180_RESET_REG
#define BMP180_RESET_VALUE        0xB6

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t bmp180_read_reg_16(i2c_dev_t *dev, uint8_t reg, int16_t *r)
{
    uint8_t d[] = { 0, 0 };

    CHECK(i2c_dev_read_reg(dev, reg, d, 2));
    *r = ((int16_t)d[0] << 8) | (d[1]);

    return ESP_OK;
}

static inline esp_err_t bmp180_start_measurement(i2c_dev_t *dev, uint8_t cmd)
{
    return i2c_dev_write_reg(dev, BMP180_CONTROL_REG, &cmd, 1);
}

static esp_err_t bmp180_get_uncompensated_temperature(i2c_dev_t *dev, int32_t *ut)
{
    // Write Start Code into reg 0xF4.
    CHECK(bmp180_start_measurement(dev, BMP180_MEASURE_TEMP));

    // Wait 5ms, datasheet states 4.5ms
    ets_delay_us(5000);

    int16_t v;
    CHECK(bmp180_read_reg_16(dev, BMP180_OUT_MSB_REG, &v));
    *ut = v;
    return ESP_OK;
}

static esp_err_t bmp180_get_uncompensated_pressure(i2c_dev_t *dev, bmp180_mode_t oss, uint32_t *up)
{
    uint16_t us;

    // Limit oss and set the measurement wait time. The datasheet
    // states 4.5, 7.5, 13.5, 25.5ms for oss 0 to 3.
    switch (oss)
    {
        case BMP180_MODE_ULTRA_LOW_POWER:                 us = 5000; break;
        case BMP180_MODE_STANDARD:                        us = 8000; break;
        case BMP180_MODE_HIGH_RESOLUTION:                 us = 14000; break;
        default: oss = BMP180_MODE_ULTRA_HIGH_RESOLUTION; us = 26000; break;
    }

    // Write Start Code into reg 0xF4
    CHECK(bmp180_start_measurement(dev, BMP180_MEASURE_PRESS | (oss << 6)));

    ets_delay_us(us);

    uint8_t d[] = { 0, 0, 0 };
    uint8_t reg = BMP180_OUT_MSB_REG;
    CHECK(i2c_dev_read_reg(dev, reg, d, 3));

    uint32_t r = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
    r >>= 8 - oss;
    *up = r;

    return ESP_OK;
}

esp_err_t bmp180_init_desc(bmp180_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = BMP180_DEVICE_ADDRESS;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t bmp180_free_desc(bmp180_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bmp180_init(bmp180_dev_t *dev)
{
    CHECK_ARG(dev);


    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t id;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BMP180_VERSION_REG, &id, 1));
    if (id != BMP180_CHIP_ID)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        ESP_LOGE(TAG, "Invalid device ID: 0x%02x", id);
        return ESP_ERR_NOT_FOUND;
    }

    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 0, &dev->AC1));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 2, &dev->AC2));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 4, &dev->AC3));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 6, (int16_t *)&dev->AC4));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 8, (int16_t *)&dev->AC5));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 10, (int16_t *)&dev->AC6));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 12, &dev->B1));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 14, &dev->B2));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 16, &dev->MB));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 18, &dev->MC));
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_read_reg_16(&dev->i2c_dev, BMP180_CALIBRATION_REG + 20, &dev->MD));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "AC1:=%d AC2:=%d AC3:=%d AC4:=%u AC5:=%u AC6:=%u", dev->AC1, dev->AC2, dev->AC3, dev->AC4, dev->AC5, dev->AC6);
    ESP_LOGD(TAG, "B1:=%d B2:=%d", dev->B1, dev->B2);
    ESP_LOGD(TAG, "MB:=%d MC:=%d MD:=%d", dev->MB, dev->MC, dev->MD);

    if (dev->AC1== 0  || dev->AC2 == 0 || dev->AC3 == 0 ||
        dev->AC4 == 0 || dev->AC5 == 0 || dev->AC6 == 0 ||
        dev->B1  == 0 || dev->B2  == 0 ||
        dev->MB  == 0 || dev->MC  == 0 || dev->MD  == 0)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t bmp180_measure(bmp180_dev_t *dev, float *temperature, uint32_t *pressure, bmp180_mode_t oss)
{
    CHECK_ARG(dev && temperature && pressure);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // Temperature is always needed, also required for pressure only.
    //
    // Calculation taken from BMP180 Datasheet
    int32_t T, P;
    int32_t UT, X1, X2, B5;
    UT = 0;
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_get_uncompensated_temperature(&dev->i2c_dev, &UT));

    X1 = ((UT - (int32_t)dev->AC6) * (int32_t)dev->AC5) >> 15;
    X2 = ((int32_t)dev->MC << 11) / (X1 + (int32_t)dev->MD);
    B5 = X1 + X2;
    T = (B5 + 8) >> 4;

    if (temperature)
        *temperature = T / 10.0;

    ESP_LOGD(TAG, "T:= %" PRIi32 ".%d", T / 10, abs(T % 10));

    if (pressure)
    {
        int32_t X3, B3, B6;
        uint32_t B4, B7, UP = 0;

        I2C_DEV_CHECK(&dev->i2c_dev, bmp180_get_uncompensated_pressure(&dev->i2c_dev, oss, &UP));

        // Calculation taken from BMP180 Datasheet
        B6 = B5 - 4000;
        X1 = ((int32_t)dev->B2 * ((B6 * B6) >> 12)) >> 11;
        X2 = ((int32_t)dev->AC2 * B6) >> 11;
        X3 = X1 + X2;

        B3 = ((((int32_t)dev->AC1 * 4 + X3) << oss) + 2) >> 2;
        X1 = ((int32_t)dev->AC3 * B6) >> 13;
        X2 = ((int32_t)dev->B1 * ((B6 * B6) >> 12)) >> 16;
        X3 = ((X1 + X2) + 2) >> 2;
        B4 = ((uint32_t)dev->AC4 * (uint32_t)(X3 + 32768)) >> 15;
        B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> oss);

        if (B7 < 0x80000000UL)
            P = (B7 * 2) / B4;
        else
            P = (B7 / B4) * 2;

        X1 = (P >> 8) * (P >> 8);
        X1 = (X1 * 3038) >> 16;
        X2 = (-7357 * P) >> 16;
        P = P + ((X1 + X2 + (int32_t)3791) >> 4);

        *pressure = P;

        ESP_LOGD(TAG, "P:= %" PRIi32, P);
    }

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

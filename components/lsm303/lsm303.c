/*
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
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
 * @file lsm303.c
 *
 * ESP-IDF Driver for LSM303: 3-axis accelerometer and magnetometer sensors
 *
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include "lsm303.h"

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>

#define I2C_FREQ_HZ          400000 // 400kHz
#define LSM303_AUTOINCREMENT 0x80

/* accelerometer registers */
#define LSM303_REG_ACC_CTRL_REG1_A     0x20
#define LSM303_REG_ACC_CTRL_REG2_A     0x21
#define LSM303_REG_ACC_CTRL_REG3_A     0x22
#define LSM303_REG_ACC_CTRL_REG4_A     0x23
#define LSM303_REG_ACC_CTRL_REG5_A     0x24
#define LSM303_REG_ACC_CTRL_REG6_A     0x25
#define LSM303_REG_ACC_REFERENCE_A     0x26
#define LSM303_REG_ACC_STATUS_REG_A    0x27
#define LSM303_REG_ACC_OUT_X_L_A       0x28
#define LSM303_REG_ACC_OUT_X_H_A       0x29
#define LSM303_REG_ACC_OUT_Y_L_A       0x2A
#define LSM303_REG_ACC_OUT_Y_H_A       0x2B
#define LSM303_REG_ACC_OUT_Z_L_A       0x2C
#define LSM303_REG_ACC_OUT_Z_H_A       0x2D
#define LSM303_REG_ACC_FIFO_CTRL_REG_A 0x2E
#define LSM303_REG_ACC_FIFO_SRC_REG_A  0x2F
#define LSM303_REG_ACC_INT1_CFG_A      0x30
#define LSM303_REG_ACC_INT1_SOURCE_A   0x31
#define LSM303_REG_ACC_INT1_THS_A      0x32
#define LSM303_REG_ACC_INT1_DURATION_A 0x33
#define LSM303_REG_ACC_INT2_CFG_A      0x34
#define LSM303_REG_ACC_INT2_SOURCE_A   0x35
#define LSM303_REG_ACC_INT2_THS_A      0x36
#define LSM303_REG_ACC_INT2_DURATION_A 0x37
#define LSM303_REG_ACC_CLICK_CFG_A     0x38
#define LSM303_REG_ACC_CLICK_SRC_A     0x39
#define LSM303_REG_ACC_CLICK_THS_A     0x3A
#define LSM303_REG_ACC_TIME_LIMIT_A    0x3B
#define LSM303_REG_ACC_TIME_LATENCY_A  0x3C
#define LSM303_REG_ACC_TIME_WINDOW_A   0x3D

#define LSM303_REG_WHO_AM_I 0x0F

/* accelerometer CTRL_REG1_A */
#define LSM303_XEN  (1 << 0) // X axis enable. Default value: 1
#define LSM303_YEN  (1 << 1) // Y axis enable. Default value: 1
#define LSM303_ZEN  (1 << 2) // Z axis enable. Default value: 1
#define LSM303_LPEN (1 << 3) // Low-power mode enable. Default value: 0

/* accelerometer CTRL_REG4_A */
#define LSM303_SIM (1 << 0) // SPI serial interface mode selection. Default value: 0
#define LSM303_HR  (1 << 3) // High resolution output mode: Default value: 0
#define LSM303_BLE (1 << 6) // Big/little endian data selection. Default value 0
#define LSM303_BDU (1 << 7) // Block data update. Default value: 0

/* accelerometer STATUS_REG_A */
#define LSM303_ACC_STATUS_ZYXOR 0x80
#define LSM303_ACC_STATUS_ZOR   0x40
#define LSM303_ACC_STATUS_YOR   0x20
#define LSM303_ACC_STATUS_XOR   0x10
#define LSM303_ACC_STATUS_ZYXDA 0x08
#define LSM303_ACC_STATUS_ZDA   0x04
#define LSM303_ACC_STATUS_YDA   0x02
#define LSM303_ACC_STATUS_XDA   0x01

#define LSM303_ACC_GRAVITY_STANDARD (9.80665f) // Earth's gravity in m/s^2

/* magnetometer registers  */
#define LSM303_REG_MAG_CRA_REG_M    0x00
#define LSM303_REG_MAG_CRB_REG_M    0x01
#define LSM303_REG_MAG_MR_REG_M     0x02
#define LSM303_REG_MAG_OUT_X_H_M    0x03
#define LSM303_REG_MAG_OUT_X_L_M    0x04
#define LSM303_REG_MAG_OUT_Z_H_M    0x05
#define LSM303_REG_MAG_OUT_Z_L_M    0x06
#define LSM303_REG_MAG_OUT_Y_H_M    0x07
#define LSM303_REG_MAG_OUT_Y_L_M    0x08
#define LSM303_REG_MAG_SR_REG_M     0x09
#define LSM303_REG_MAG_IRA_REG_M    0x0A
#define LSM303_REG_MAG_IRB_REG_M    0x0B
#define LSM303_REG_MAG_IRC_REG_M    0x0C
#define LSM303_REG_MAG_TEMP_OUT_H_M 0x31
#define LSM303_REG_MAG_TEMP_OUT_L_M 0x32

/* magnetometer REG_MAG_CRA_REG_M */
#define LSM303_TEMP_EN 0x80

/* magnetometer REG_SR_REG_M */
#define LSM303_MAG_STATUS_LOCK 0x02
#define LSM303_MAG_STATUS_DRDY 0x01

/* mag data order: high before low (different than accel) */
#define LSM303_MAG_XH 0
#define LSM303_MAG_XL 1
#define LSM303_MAG_ZH 2
#define LSM303_MAG_ZL 3
#define LSM303_MAG_YH 4
#define LSM303_MAG_YL 5

#define LSM303_MAG_GAUSS_TO_MICROTESLA (100.0f) // Gauss to micro-Tesla multiplier

static const char *TAG = "lsm303";

#define CHECK(x)                           \
    do                                     \
    {                                      \
        esp_err_t __;                      \
        if ((__ = x) != ESP_OK)            \
            return __;                     \
    }                                      \
    while (0)
#define CHECK_ARG(VAL)                     \
    do                                     \
    {                                      \
        if (!(VAL))                        \
            return ESP_ERR_INVALID_ARG;    \
    }                                      \
    while (0)

inline static esp_err_t read_acc_reg_nolock(lsm303_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(&dev->i2c_dev_acc, reg, val, 1);
}

inline static esp_err_t read_mag_reg_nolock(lsm303_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(&dev->i2c_dev_mag, reg, val, 1);
}

esp_err_t lsm303_init_desc(lsm303_t *dev, uint8_t acc_addr, uint8_t mag_addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev_acc.port = port;
    dev->i2c_dev_acc.addr = acc_addr;
    dev->i2c_dev_acc.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev_acc.cfg.scl_io_num = scl_gpio;

    dev->i2c_dev_mag.port = port;
    dev->i2c_dev_mag.addr = mag_addr;
    dev->i2c_dev_mag.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev_mag.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev_acc.cfg.master.clk_speed = I2C_FREQ_HZ;
    dev->i2c_dev_mag.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    dev->acc_mode = LSM303_ACC_MODE_NORMAL;
    dev->acc_rate = LSM303_ODR_10_HZ;
    dev->acc_scale = LSM303_ACC_SCALE_2G;

    dev->mag_mode = LSM303_MAG_MODE_CONT;
    dev->mag_rate = LSM303_MAG_RATE_3_0;
    dev->mag_gain = LSM303_MAG_GAIN_1_3;

    CHECK(i2c_dev_create_mutex(&dev->i2c_dev_acc));
    CHECK(i2c_dev_create_mutex(&dev->i2c_dev_mag));
    return ESP_OK;
}

esp_err_t lsm303_free_desc(lsm303_t *dev)
{
    CHECK_ARG(dev);
    CHECK(i2c_dev_delete_mutex(&dev->i2c_dev_acc));
    CHECK(i2c_dev_delete_mutex(&dev->i2c_dev_mag));
    return ESP_OK;
}

esp_err_t lsm303_init(lsm303_t *dev)
{
    CHECK_ARG(dev);
    uint8_t v = 0;

    /* Check accelerometer connection */
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_acc);
    I2C_DEV_CHECK(&dev->i2c_dev_acc, i2c_dev_read_reg(&dev->i2c_dev_acc, LSM303_REG_WHO_AM_I, &v, 1));

    if (v != 0x33)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_acc);
        ESP_LOGE(TAG, "Unknown acc ID: 0x%02x", v);
        return ESP_ERR_NOT_FOUND;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_acc);

    /* Check magnetometer connection */
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_mag);
    I2C_DEV_CHECK(&dev->i2c_dev_mag, i2c_dev_read_reg(&dev->i2c_dev_mag, LSM303_REG_MAG_IRA_REG_M, &v, 1));

    if (v != 0x48)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_mag);
        ESP_LOGE(TAG, "Unknown mag ID: 0x%02x", v);
        return ESP_ERR_NOT_FOUND;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_mag);

    /* Initialize accelerometer */
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_acc);

    /* setup CTRL_REG1
        [7:4] ODR - Output data rate
        [3] LPen - Low Power mode
        [2] ZEN - Z axis enable
        [1] YEN - Y axis enable
        [0] XEN - X axis enable
    */
    v = 0x00;
    v |= LSM303_XEN | LSM303_YEN | LSM303_ZEN;
    v |= (dev->acc_rate << 4);
    v |= (dev->acc_mode == LSM303_ACC_MODE_LOW_POWER) ? LSM303_LPEN : 0x00;
    I2C_DEV_CHECK(&dev->i2c_dev_acc, i2c_dev_write_reg(&dev->i2c_dev_acc, LSM303_REG_ACC_CTRL_REG1_A, &v, 1));

    /* setup CTRL_REG4
        [7]		BDU			- Block Data Update
        [6]		BLE			- Big/Little Endian Data Selection
        [5:4]	FS1 FS0		- Full Scale selection
        [3]		HR			- High Resolution
        [2:1]	ST1 ST0		- Self Test Enable
        [0]		SIM			- SIM SPI Serial Interface Mode selection

    */
    v = 0x00;
    v |= LSM303_BDU;
    v |= (dev->acc_scale << 4);
    v |= (dev->acc_mode == LSM303_ACC_MODE_HIGH_RESOLUTION) ? LSM303_HR : 0x00;

    I2C_DEV_CHECK(&dev->i2c_dev_acc, i2c_dev_write_reg(&dev->i2c_dev_acc, LSM303_REG_ACC_CTRL_REG4_A, &v, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_acc);

    /* Initialize magnetometer */
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_mag);

    /* setup LSM303_REG_MAG_CRA_REG_M
        [7]   TEMP_EN - Temperature sensor enable
        [6:5] reserved
        [4:2] DO = Data output rate bits
        [1:0] reserved
    */
    v = 0x00;
    v |= LSM303_TEMP_EN;
    v |= (dev->mag_rate << 2);
    I2C_DEV_CHECK(&dev->i2c_dev_acc, i2c_dev_write_reg(&dev->i2c_dev_mag, LSM303_REG_MAG_CRA_REG_M, &v, 1));

    /* setup LSM303_REG_MAG_CRB_REG_M
        [7:5] GN - Gain configuration bits
        [4:0] reserved
    */
    v = 0x00;
    v |= (dev->mag_gain << 5);
    I2C_DEV_CHECK(&dev->i2c_dev_acc, i2c_dev_write_reg(&dev->i2c_dev_mag, LSM303_REG_MAG_CRB_REG_M, &v, 1));

    /* setup LSM303_REG_MAG_MR_REG_M
        [7:2] reserved
        [1:0] Mode select bits
    */
    v = 0x00;
    I2C_DEV_CHECK(&dev->i2c_dev_acc, i2c_dev_write_reg(&dev->i2c_dev_mag, LSM303_REG_MAG_MR_REG_M, &v, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_mag);
    return ESP_OK;
}

esp_err_t lsm303_acc_set_config(lsm303_t *dev, lsm303_acc_mode_t mode, lsm303_acc_rate_t rate, lsm303_acc_scale_t scale)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_acc);
    dev->acc_mode = mode;
    dev->acc_rate = rate;
    dev->acc_scale = scale;
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_acc);
    return lsm303_init(dev);
}

esp_err_t lsm303_acc_get_config(lsm303_t *dev, lsm303_acc_mode_t *mode, lsm303_acc_rate_t *rate, lsm303_acc_scale_t *scale)
{
    CHECK_ARG(dev && mode && rate && scale);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_acc);
    *mode = dev->acc_mode;
    *rate = dev->acc_rate;
    *scale = dev->acc_scale;
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_acc);
    return ESP_OK;
}

esp_err_t lsm303_acc_data_ready(lsm303_t *dev, bool *ready)
{
    CHECK_ARG(dev && ready);

    uint8_t v;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_acc);
    I2C_DEV_CHECK(&dev->i2c_dev_acc, read_acc_reg_nolock(dev, LSM303_REG_ACC_STATUS_REG_A, &v));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_acc);
    *ready = v & LSM303_ACC_STATUS_ZYXDA;
    return ESP_OK;
}

esp_err_t lsm303_acc_get_raw_data(lsm303_t *dev, lsm303_acc_raw_data_t *raw)
{
    CHECK_ARG(dev && raw);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_acc);
    esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev_acc, LSM303_REG_ACC_OUT_X_L_A | LSM303_AUTOINCREMENT, raw, 6);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not read data register, err = %d", ret);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_acc);
    return ret;
}

esp_err_t lsm303_acc_raw_to_g(lsm303_t *dev, lsm303_acc_raw_data_t *raw, lsm303_acc_data_t *data)
{
    CHECK_ARG(dev && raw && data);

    static const float lsb[][4] = {
        [LSM303_ACC_MODE_NORMAL] = {
            [LSM303_ACC_SCALE_2G] = 0.0039,
            [LSM303_ACC_SCALE_4G] = 0.00782,
            [LSM303_ACC_SCALE_8G] = 0.01563,
            [LSM303_ACC_SCALE_16G] = 0.0469
        },
        [LSM303_ACC_MODE_HIGH_RESOLUTION] = {
            [LSM303_ACC_SCALE_2G] = 0.00098,
            [LSM303_ACC_SCALE_4G] = 0.00195,
            [LSM303_ACC_SCALE_8G] = 0.0039,
            [LSM303_ACC_SCALE_16G] = 0.01172
        },
        [LSM303_ACC_MODE_LOW_POWER] = {
            [LSM303_ACC_SCALE_2G] = 0.01563,
            [LSM303_ACC_SCALE_4G] = 0.03126,
            [LSM303_ACC_SCALE_8G] = 0.06252,
            [LSM303_ACC_SCALE_16G] = 0.18758
        },
    };
    static const int shift[] = {
        [LSM303_ACC_MODE_NORMAL] = 6,          // 10-bit
        [LSM303_ACC_MODE_HIGH_RESOLUTION] = 4, // 12-bit
        [LSM303_ACC_MODE_LOW_POWER] = 8        // 8-bit
    };
    data->x = (raw->x >> shift[dev->acc_mode]) * lsb[dev->acc_mode][dev->acc_scale];
    data->y = (raw->y >> shift[dev->acc_mode]) * lsb[dev->acc_mode][dev->acc_scale];
    data->z = (raw->z >> shift[dev->acc_mode]) * lsb[dev->acc_mode][dev->acc_scale];
    return ESP_OK;
}

esp_err_t lsm303_acc_get_data(lsm303_t *dev, lsm303_acc_data_t *data)
{
    CHECK_ARG(dev && data);

    lsm303_acc_raw_data_t raw;
    CHECK(lsm303_acc_get_raw_data(dev, &raw));
    return lsm303_acc_raw_to_g(dev, &raw, data);
}

esp_err_t lsm303_mag_set_config(lsm303_t *dev, lsm303_mag_mode_t mode, lsm303_mag_rate_t rate, lsm303_mag_gain_t gain)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_mag);
    dev->mag_mode = mode;
    dev->mag_rate = rate;
    dev->mag_gain = gain;
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_mag);
    return lsm303_init(dev);
}

esp_err_t lsm303_mag_get_config(lsm303_t *dev, lsm303_mag_mode_t *mode, lsm303_mag_rate_t *rate, lsm303_mag_gain_t *gain)
{
    CHECK_ARG(dev && mode && rate && gain);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_mag);
    *mode = dev->mag_mode;
    *rate = dev->mag_rate;
    *gain = dev->mag_gain;
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_mag);
    return ESP_OK;
}

esp_err_t lsm303_mag_data_ready(lsm303_t *dev, bool *ready)
{
    CHECK_ARG(dev && ready);

    uint8_t v;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_mag);
    I2C_DEV_CHECK(&dev->i2c_dev_mag, read_mag_reg_nolock(dev, LSM303_REG_MAG_SR_REG_M, &v));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_mag);
    *ready = v & LSM303_MAG_STATUS_DRDY;
    return ESP_OK;
}

esp_err_t lsm303_mag_get_raw_data(lsm303_t *dev, lsm303_mag_raw_data_t *raw)
{
    CHECK_ARG(dev && raw);
    uint8_t buf[6] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_mag);
    esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev_mag, LSM303_REG_MAG_OUT_X_H_M | LSM303_AUTOINCREMENT, buf, 6);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not read data register, err = %d", ret);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_mag);

    raw->x = (int16_t)(buf[LSM303_MAG_XL] | (buf[LSM303_MAG_XH] << 8));
    raw->y = (int16_t)(buf[LSM303_MAG_YL] | (buf[LSM303_MAG_YH] << 8));
    raw->z = (int16_t)(buf[LSM303_MAG_ZL] | (buf[LSM303_MAG_ZH] << 8));
    return ret;
}

esp_err_t lsm303_mag_raw_to_uT(lsm303_t *dev, lsm303_mag_raw_data_t *raw, lsm303_mag_data_t *data)
{
    CHECK_ARG(dev && raw && data);
    /* gain for XY axis is different from Z axis */
    enum { GAIN_XY = 0, GAIN_Z = 1 };
    /*  { xy , z} */
    static const float gauss_lsb[][2] = {
        [LSM303_MAG_GAIN_1_3] = { 1100, 980 },
        [LSM303_MAG_GAIN_1_9] = { 855, 760 },
        [LSM303_MAG_GAIN_2_5] = { 670, 600 },
        [LSM303_MAG_GAIN_4_0] = { 450, 400 },
        [LSM303_MAG_GAIN_4_7] = { 400, 355 },
        [LSM303_MAG_GAIN_5_6] = { 330, 295 },
        [LSM303_MAG_GAIN_8_1] = { 230, 205 },
    };

    data->x = (float)raw->x / gauss_lsb[dev->mag_gain][GAIN_XY] * LSM303_MAG_GAUSS_TO_MICROTESLA;
    data->y = (float)raw->y / gauss_lsb[dev->mag_gain][GAIN_XY] * LSM303_MAG_GAUSS_TO_MICROTESLA;
    data->z = (float)raw->z / gauss_lsb[dev->mag_gain][GAIN_Z] * LSM303_MAG_GAUSS_TO_MICROTESLA;
    return ESP_OK;
}

esp_err_t lsm303_mag_get_data(lsm303_t *dev, lsm303_mag_data_t *data)
{
    CHECK_ARG(dev && data);

    lsm303_mag_raw_data_t raw;
    CHECK(lsm303_mag_get_raw_data(dev, &raw));
    return lsm303_mag_raw_to_uT(dev, &raw, data);
}

esp_err_t lsm303_mag_get_temp(lsm303_t *dev, float *temp)
{
    CHECK_ARG(dev && temp);
    uint8_t buf[2] = { 0 };
    int16_t t;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev_mag);
    esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev_mag, LSM303_REG_MAG_TEMP_OUT_H_M | LSM303_AUTOINCREMENT, buf, 2);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not read TEMP_OUT register, err = %d", ret);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev_mag);
    t = (((int16_t)((buf[0] << 8) | buf[1])) >> 4) + (20 << 3); // 12bit and 20*C
    *temp = (float)t / 10;
    return ret;
}

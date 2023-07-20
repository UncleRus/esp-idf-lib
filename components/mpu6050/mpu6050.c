/*
 * The MIT License (MIT)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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
 *
 */

/**
 * @file mpu6050.c
 *
 * ESP-IDF driver for MPU6050 MEMS Sensor.
 *
 * 6-axis motion tracking devices designed for the low power, low cost,
 * and high performance requirements of smartphones, tablets and wearable sensors.
 *
 * Copyright (c) 2012 Jeff Rowberg <https://www.i2cdevlib.com/>
 * Copyright (c) 2019 Angelo Elias Dalzotto <150633@upf.br>
 * Copyright (c) 2019 Gabriel Boni Vicari <133192@upf.br>
 * Copyright (c) 2019 GEPID - Grupo de Pesquisa em Cultura Digital <http://gepid.upf.br/>
 * Copyright (c) 2023 Raghav Jha <https://github.com/horsemann07>
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
 */

#include "mpu6050.h"
#include "mpu6050_regs.h"
#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Max 1MHz for esp-idf, but device supports up to 1.7Mhz
#define I2C_FREQ_HZ (1000000)

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "mpu6050";

static const float accel_res[] = {
    [MPU6050_ACCEL_RANGE_2]  = 2.0f / 32768.0f,
    [MPU6050_ACCEL_RANGE_4]  = 4.0f / 32768.0f,
    [MPU6050_ACCEL_RANGE_8]  = 8.0f / 32768.0f,
    [MPU6050_ACCEL_RANGE_16] = 16.0f / 32768.0f,
};

static const float gyro_res[] = {
    [MPU6050_GYRO_RANGE_250]  = 250.0f / 32768.0f,
    [MPU6050_GYRO_RANGE_500]  = 500.0f / 32768.0f,
    [MPU6050_GYRO_RANGE_1000] = 1000.0f / 32768.0f,
    [MPU6050_GYRO_RANGE_2000] = 2000.0f / 32768.0f,
};

inline static float get_accel_value(mpu6050_dev_t *dev, int16_t raw)
{
    return (float)raw * accel_res[dev->ranges.accel];
}

inline static float get_gyro_value(mpu6050_dev_t *dev, int16_t raw)
{
    return (float)raw * gyro_res[dev->ranges.gyro];
}

inline static int16_t shuffle(uint16_t word)
{
    return (int16_t)((word >> 8) | (word << 8));
}

inline static uint16_t ushuffle(uint16_t word)
{
    return ((word >> 8) | (word << 8));
}

static esp_err_t read_reg_bits(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t offset, uint8_t mask, uint8_t *value)
{
    CHECK_ARG(dev && value && mask);

    uint8_t buf;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, &buf, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *value = (buf & mask) >> offset;

    return ESP_OK;
}

static esp_err_t write_reg_bits(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t offset, uint8_t mask, uint8_t value)
{
    CHECK_ARG(dev && mask);

    uint8_t buf;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, &buf, 1));

    buf = (buf & ~mask) | ((value << offset) & mask);

    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg_addr, &buf, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_reg(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t *value)
{
    CHECK_ARG(dev && value);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, value, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t write_reg(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t data)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg_addr, &data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_reg_word(mpu6050_dev_t *dev, uint8_t reg_addr, int16_t *value)
{
    CHECK_ARG(dev && value);

    uint16_t buf;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, &buf, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *value = shuffle(buf);

    return ESP_OK;
}

static esp_err_t write_reg_word(mpu6050_dev_t *dev, uint8_t reg_addr, int16_t value)
{
    CHECK_ARG(dev);

    uint16_t buf = ushuffle(value);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, &buf, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_reg_bool(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t offset, bool *value)
{
    CHECK_ARG(value);

    uint8_t buf;
    CHECK(read_reg_bits(dev, reg_addr, offset, BIT(offset), &buf));
    *value = buf != 0 ? true : false;

    return ESP_OK;
}

static inline esp_err_t write_reg_bool(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t offset, bool value)
{
    return write_reg_bits(dev, reg_addr, offset, BIT(offset), value ? 1 : 0);
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t mpu6050_init_desc(mpu6050_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != MPU6050_I2C_ADDRESS_LOW && addr != MPU6050_I2C_ADDRESS_HIGH)
    {
        ESP_LOGE(TAG, "Invalid device address: 0x%02x", addr);
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

esp_err_t mpu6050_free_desc(mpu6050_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t mpu6050_init(mpu6050_dev_t *dev)
{
    CHECK_ARG(dev);

    CHECK(mpu6050_set_clock_source(dev, MPU6050_CLOCK_PLL_X));
    CHECK(mpu6050_set_full_scale_gyro_range(dev, MPU6050_GYRO_RANGE_250));
    CHECK(mpu6050_set_full_scale_accel_range(dev, MPU6050_ACCEL_RANGE_2));
    CHECK(mpu6050_set_sleep_enabled(dev, false));

    return ESP_OK;
}

esp_err_t mpu6050_get_aux_vddio_level(mpu6050_dev_t *dev, mpu6050_vddio_level_t *level)
{
    return read_reg_bits(dev, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, BIT(MPU6050_TC_PWR_MODE_BIT), (uint8_t *)level);
}

esp_err_t mpu6050_set_aux_vddio_level(mpu6050_dev_t *dev, mpu6050_vddio_level_t level)
{
    return write_reg_bits(dev, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, BIT(MPU6050_TC_PWR_MODE_BIT), level);
}

esp_err_t mpu6050_get_rate(mpu6050_dev_t *dev, uint8_t *rate)
{
    return read_reg(dev, MPU6050_REGISTER_SMPLRT_DIV, rate);
}

esp_err_t mpu6050_set_rate(mpu6050_dev_t *dev, uint8_t rate)
{
    return write_reg(dev, MPU6050_REGISTER_SMPLRT_DIV, rate);
}

esp_err_t mpu6050_get_external_frame_sync(mpu6050_dev_t *dev, mpu6050_ext_sync_t *sync)
{
    return read_reg_bits(dev, MPU6050_REGISTER_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_MASK, (uint8_t *)sync);
}

esp_err_t mpu6050_set_external_frame_sync(mpu6050_dev_t *dev, mpu6050_ext_sync_t sync)
{
    return write_reg_bits(dev, MPU6050_REGISTER_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_MASK, sync);
}

esp_err_t mpu6050_get_dlpf_mode(mpu6050_dev_t *dev, mpu6050_dlpf_mode_t *mode)
{
    return read_reg_bits(dev, MPU6050_REGISTER_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_MASK, (uint8_t *)mode);
}

esp_err_t mpu6050_set_dlpf_mode(mpu6050_dev_t *dev, mpu6050_dlpf_mode_t mode)
{
    return write_reg_bits(dev, MPU6050_REGISTER_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_MASK, mode);
}

esp_err_t mpu6050_get_full_scale_gyro_range(mpu6050_dev_t *dev, mpu6050_gyro_range_t *range)
{
    esp_err_t res = read_reg_bits(dev, MPU6050_REGISTER_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_MASK, (uint8_t *)range);
    if (res == ESP_OK)
        dev->ranges.gyro = *range;

    return res;
}

esp_err_t mpu6050_set_full_scale_gyro_range(mpu6050_dev_t *dev, mpu6050_gyro_range_t range)
{
    CHECK_ARG(range <= MPU6050_GYRO_RANGE_2000);

    esp_err_t res = write_reg_bits(dev, MPU6050_REGISTER_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_MASK, range);
    if (res == ESP_OK)
        dev->ranges.gyro = range;

    return res;
}

esp_err_t mpu6050_get_accel_self_test_factory_trim(mpu6050_dev_t *dev, mpu6050_axis_t axis, uint8_t *trim)
{
    CHECK_ARG(dev && trim && axis <= MPU6050_Z_AXIS);

    static const struct { uint8_t r, s; } regs[] = {
        [MPU6050_X_AXIS] = { .r = MPU6050_REGISTER_SELF_TEST_X, .s = 4 },
        [MPU6050_Y_AXIS] = { .r = MPU6050_REGISTER_SELF_TEST_Y, .s = 2 },
        [MPU6050_Z_AXIS] = { .r = MPU6050_REGISTER_SELF_TEST_Z, .s = 0 },
    };

    uint8_t a = 0;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, regs[axis].r, trim, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MPU6050_REGISTER_SELF_TEST_A, &a, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *trim = ((*trim) >> 3) | ((a >> regs[axis].s) & 0x03);

    return ESP_OK;
}

esp_err_t mpu6050_get_gyro_self_test_factory_trim(mpu6050_dev_t *dev, mpu6050_axis_t axis, uint8_t *trim)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    static const uint8_t regs[] = {
        [MPU6050_X_AXIS] = MPU6050_REGISTER_SELF_TEST_X,
        [MPU6050_Y_AXIS] = MPU6050_REGISTER_SELF_TEST_Y,
        [MPU6050_Z_AXIS] = MPU6050_REGISTER_SELF_TEST_Z,
    };

    CHECK(read_reg(dev, regs[axis], trim));
    *trim = *trim & 0x1f;

    return ESP_OK;
}

static const uint8_t accel_sta_bits[] = {
    [MPU6050_X_AXIS] = MPU6050_ACONFIG_XA_ST_BIT,
    [MPU6050_Y_AXIS] = MPU6050_ACONFIG_YA_ST_BIT,
    [MPU6050_Z_AXIS] = MPU6050_ACONFIG_ZA_ST_BIT,
};

esp_err_t mpu6050_get_accel_self_test(mpu6050_dev_t *dev, mpu6050_axis_t axis, bool *enabled)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return read_reg_bool(dev, MPU6050_REGISTER_ACCEL_CONFIG, accel_sta_bits[axis], enabled);
}

esp_err_t mpu6050_set_accel_self_test(mpu6050_dev_t *dev, mpu6050_axis_t axis, bool enabled)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return write_reg_bool(dev, MPU6050_REGISTER_ACCEL_CONFIG, accel_sta_bits[axis], enabled);
}

esp_err_t mpu6050_get_full_scale_accel_range(mpu6050_dev_t *dev, mpu6050_accel_range_t *range)
{
    esp_err_t res = read_reg_bits(dev, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_MASK, (uint8_t *)range);
    if (res == ESP_OK)
        dev->ranges.accel = *range;

    return res;
}

esp_err_t mpu6050_set_full_scale_accel_range(mpu6050_dev_t *dev, mpu6050_accel_range_t range)
{
    CHECK_ARG(range <= MPU6050_ACCEL_RANGE_16);

    esp_err_t res = write_reg_bits(dev, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_MASK, range);
    if (res == ESP_OK)
        dev->ranges.accel = range;

    return res;
}

esp_err_t mpu6050_get_dhpf_mode(mpu6050_dev_t *dev, mpu6050_dhpf_mode_t *mode)
{
    return read_reg_bits(dev, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_MASK, (uint8_t *)mode);
}

esp_err_t mpu6050_set_dhpf_mode(mpu6050_dev_t *dev, mpu6050_dhpf_mode_t mode)
{
    return write_reg_bits(dev, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_MASK, mode);
}

esp_err_t mpu6050_get_freefall_detection_threshold(mpu6050_dev_t *dev, uint8_t *threshold)
{
    return read_reg(dev, MPU6050_REGISTER_FF_THR, threshold);
}

esp_err_t mpu6050_set_freefall_detection_threshold(mpu6050_dev_t *dev, uint8_t threshold)
{
    return write_reg(dev, MPU6050_REGISTER_FF_THR, threshold);
}

esp_err_t mpu6050_get_freefall_detection_duration(mpu6050_dev_t *dev, uint8_t *duration_ms)
{
    return read_reg(dev, MPU6050_REGISTER_FF_DUR, duration_ms);
}

esp_err_t mpu6050_set_freefall_detection_duration(mpu6050_dev_t *dev, uint8_t duration_ms)
{
    return write_reg(dev, MPU6050_REGISTER_FF_DUR, duration_ms);
}

esp_err_t mpu6050_get_motion_detection_threshold(mpu6050_dev_t *dev, uint8_t *threshold)
{
    return read_reg(dev, MPU6050_REGISTER_MOT_THR, threshold);
}

esp_err_t mpu6050_set_motion_detection_threshold(mpu6050_dev_t *dev, uint8_t threshold)
{
    return write_reg(dev, MPU6050_REGISTER_MOT_THR, threshold);
}

esp_err_t mpu6050_get_motion_detection_duration(mpu6050_dev_t *dev, uint8_t *duration)
{
    return read_reg(dev, MPU6050_REGISTER_MOT_DUR, duration);
}

esp_err_t mpu6050_set_motion_detection_duration(mpu6050_dev_t *dev, uint8_t duration)
{
    return write_reg(dev, MPU6050_REGISTER_MOT_DUR, duration);
}

esp_err_t mpu6050_get_zero_motion_detection_threshold(mpu6050_dev_t *dev, uint8_t *threshold)
{
    return read_reg(dev, MPU6050_REGISTER_ZRMOT_THR, threshold);
}

esp_err_t mpu6050_set_zero_motion_detection_threshold(mpu6050_dev_t *dev, uint8_t threshold)
{
    return write_reg(dev, MPU6050_REGISTER_ZRMOT_THR, threshold);
}

esp_err_t mpu6050_get_zero_motion_detection_duration(mpu6050_dev_t *dev, uint8_t *duration)
{
    return read_reg(dev, MPU6050_REGISTER_ZRMOT_DUR, duration);
}

esp_err_t mpu6050_set_zero_motion_detection_duration(mpu6050_dev_t *dev, uint8_t duration)
{
    return write_reg(dev, MPU6050_REGISTER_ZRMOT_DUR, duration);
}

esp_err_t mpu6050_get_temp_fifo_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}

esp_err_t mpu6050_set_temp_fifo_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}

static const uint8_t gyro_fifo_bits[] = {
    [MPU6050_X_AXIS] = MPU6050_XG_FIFO_EN_BIT,
    [MPU6050_Y_AXIS] = MPU6050_YG_FIFO_EN_BIT,
    [MPU6050_Z_AXIS] = MPU6050_ZG_FIFO_EN_BIT,
};

esp_err_t mpu6050_get_gyro_fifo_enabled(mpu6050_dev_t *dev, mpu6050_axis_t axis, bool *enabled)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return read_reg_bool(dev, MPU6050_REGISTER_FIFO_EN, gyro_fifo_bits[axis], enabled);
}

esp_err_t mpu6050_set_gyro_fifo_enabled(mpu6050_dev_t *dev, mpu6050_axis_t axis, bool enabled)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return write_reg_bool(dev, MPU6050_REGISTER_FIFO_EN, gyro_fifo_bits[axis], enabled);
}

esp_err_t mpu6050_get_accel_fifo_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}

esp_err_t mpu6050_set_accel_fifo_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}

static const struct { uint8_t r, b; } slave_fifo_bits[] = {
    [MPU6050_SLAVE_0] = { .r = MPU6050_REGISTER_FIFO_EN, .b = MPU6050_SLV2_FIFO_EN_BIT },
    [MPU6050_SLAVE_1] = { .r = MPU6050_REGISTER_FIFO_EN, .b = MPU6050_SLV1_FIFO_EN_BIT },
    [MPU6050_SLAVE_2] = { .r = MPU6050_REGISTER_FIFO_EN, .b = MPU6050_SLV0_FIFO_EN_BIT },
    [MPU6050_SLAVE_3] = { .r = MPU6050_REGISTER_I2C_MST_CTRL, .b = MPU6050_SLV_3_FIFO_EN_BIT },
};

esp_err_t mpu6050_get_slave_fifo_enabled(mpu6050_dev_t *dev, mpu6050_slave_t num, bool *enabled)
{
    CHECK_ARG(num < MPU6050_SLAVE_4);

    return read_reg_bool(dev, slave_fifo_bits[num].r, slave_fifo_bits[num].b, enabled);
}

esp_err_t mpu6050_set_slave_fifo_enabled(mpu6050_dev_t *dev, mpu6050_slave_t num, bool enabled)
{
    CHECK_ARG(num < MPU6050_SLAVE_4);

    return write_reg_bool(dev, slave_fifo_bits[num].r, slave_fifo_bits[num].b, enabled);
}

esp_err_t mpu6050_get_multi_master_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, enabled);
}

esp_err_t mpu6050_set_multi_master_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, enabled);
}

esp_err_t mpu6050_get_wait_for_external_sensor_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}

esp_err_t mpu6050_set_wait_for_external_sensor_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}

esp_err_t mpu6050_get_slave_read_write_transition_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, enabled);
}

esp_err_t mpu6050_set_slave_read_write_transition_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, enabled);
}

esp_err_t mpu6050_get_master_clock_speed(mpu6050_dev_t *dev, mpu6050_i2c_master_clock_t *clk_spd)
{
    return read_reg_bits(dev, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_MASK, (uint8_t *)clk_spd);
}

esp_err_t mpu6050_set_master_clock_speed(mpu6050_dev_t *dev, mpu6050_i2c_master_clock_t clk_spd)
{
    return write_reg_bits(dev, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_MASK, clk_spd);
}

esp_err_t mpu6050_get_slave_address(mpu6050_dev_t *dev, mpu6050_slave_t num, uint8_t *addr)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return read_reg(dev, MPU6050_REGISTER_I2C_SLV0_ADDR + num * 3, addr);
}

esp_err_t mpu6050_set_slave_address(mpu6050_dev_t *dev, mpu6050_slave_t num, uint8_t address)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return write_reg(dev, MPU6050_REGISTER_I2C_SLV0_ADDR + num * 3, address);
}

esp_err_t mpu6050_get_slave_register(mpu6050_dev_t *dev, mpu6050_slave_t num, uint8_t *reg)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return read_reg(dev, MPU6050_REGISTER_I2C_SLV0_REG + num * 3, reg);
}

esp_err_t mpu6050_set_slave_register(mpu6050_dev_t *dev, mpu6050_slave_t num, uint8_t reg)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return write_reg(dev, MPU6050_REGISTER_I2C_SLV0_REG + num * 3, reg);
}

esp_err_t mpu6050_get_slave_enabled(mpu6050_dev_t *dev, mpu6050_slave_t num, bool *enabled)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return read_reg_bool(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT, enabled);
}

esp_err_t mpu6050_set_slave_enabled(mpu6050_dev_t *dev, mpu6050_slave_t num, bool enabled)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return write_reg_bool(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT, enabled);
}

esp_err_t mpu6050_get_slave_word_byte_swap(mpu6050_dev_t *dev, mpu6050_slave_t num, bool *enabled)
{
    CHECK_ARG(num < MPU6050_SLAVE_4);

    return read_reg_bool(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_BYTE_SW_BIT, enabled);
}

esp_err_t mpu6050_set_slave_word_byte_swap(mpu6050_dev_t *dev, mpu6050_slave_t num, bool enabled)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return write_reg_bool(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_BYTE_SW_BIT, enabled);
}

esp_err_t mpu6050_get_slave_write_mode(mpu6050_dev_t *dev, mpu6050_slave_t num, bool *mode)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return read_reg_bool(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_REG_DIS_BIT, mode);
}

esp_err_t mpu6050_set_slave_write_mode(mpu6050_dev_t *dev, mpu6050_slave_t num, bool mode)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return write_reg_bool(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_REG_DIS_BIT, mode);
}

esp_err_t mpu6050_get_slave_word_group_offset(mpu6050_dev_t *dev, mpu6050_slave_t num, bool *enabled)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return read_reg_bool(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_GRP_BIT, enabled);
}

esp_err_t mpu6050_set_slave_word_group_offset(mpu6050_dev_t *dev, mpu6050_slave_t num, bool enabled)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return write_reg_bool(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_GRP_BIT, enabled);
}

esp_err_t mpu6050_get_slave_data_length(mpu6050_dev_t *dev, mpu6050_slave_t num, uint8_t *length)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return read_reg_bits(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_MASK, length);
}

esp_err_t mpu6050_set_slave_data_length(mpu6050_dev_t *dev, mpu6050_slave_t num, uint8_t length)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return write_reg_bits(dev, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_MASK, length);
}

esp_err_t mpu6050_set_slave_4_output_byte(mpu6050_dev_t *dev, uint8_t data)
{
    return write_reg(dev, MPU6050_REGISTER_I2C_SLV4_DO, data);
}

esp_err_t mpu6050_get_slave_4_interrupt_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, enabled);
}

esp_err_t mpu6050_set_slave_4_interrupt_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, enabled);
}

esp_err_t mpu6050_get_slave_4_master_delay(mpu6050_dev_t *dev, uint8_t *delay)
{
    return read_reg_bits(dev, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT, MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}

esp_err_t mpu6050_set_slave_4_master_delay(mpu6050_dev_t *dev, uint8_t delay)
{
    return write_reg_bits(dev, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT, MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}

esp_err_t mpu6050_get_slave_4_input_byte(mpu6050_dev_t *dev, uint8_t *byte)
{
    return read_reg(dev, MPU6050_REGISTER_I2C_SLV4_DI, byte);
}

esp_err_t mpu6050_get_passthrough_status(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_PASS_THROUGH_BIT, enabled);
}

esp_err_t mpu6050_get_slave_4_is_done(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_DONE_BIT, enabled);
}

esp_err_t mpu6050_get_lost_arbitration(mpu6050_dev_t *dev, bool *lost)
{
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_LOST_ARB_BIT, lost);
}

esp_err_t mpu6050_get_slave_nack(mpu6050_dev_t *dev, mpu6050_slave_t num, bool *nack)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    static const uint8_t bits[] = {
        [MPU6050_SLAVE_0] = MPU6050_MST_I2C_SLV0_NACK_BIT,
        [MPU6050_SLAVE_1] = MPU6050_MST_I2C_SLV1_NACK_BIT,
        [MPU6050_SLAVE_2] = MPU6050_MST_I2C_SLV2_NACK_BIT,
        [MPU6050_SLAVE_3] = MPU6050_MST_I2C_SLV3_NACK_BIT,
        [MPU6050_SLAVE_4] = MPU6050_MST_I2C_SLV4_NACK_BIT,
    };
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_STATUS, bits[num], nack);
}

esp_err_t mpu6050_get_interrupt_mode(mpu6050_dev_t *dev, mpu6050_int_level_t *mode)
{
    return read_reg_bits(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, BIT(MPU6050_INTCFG_INT_LEVEL_BIT), (uint8_t *)mode);
}

esp_err_t mpu6050_set_interrupt_mode(mpu6050_dev_t *dev, mpu6050_int_level_t mode)
{
    CHECK_ARG(mode <= MPU6050_INT_LEVEL_LOW);

    return write_reg_bits(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, BIT(MPU6050_INTCFG_INT_LEVEL_BIT), mode);
}

esp_err_t mpu6050_get_interrupt_drive(mpu6050_dev_t *dev, mpu6050_int_drive_t *drive)
{
    return read_reg_bits(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, BIT(MPU6050_INTCFG_INT_OPEN_BIT), (uint8_t *)drive);
}

esp_err_t mpu6050_set_interrupt_drive(mpu6050_dev_t *dev, mpu6050_int_drive_t drive)
{
    CHECK_ARG(drive <= MPU6050_INT_OPEN_DRAIN);

    return write_reg_bits(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, BIT(MPU6050_INTCFG_INT_OPEN_BIT), drive);
}

esp_err_t mpu6050_get_interrupt_latch(mpu6050_dev_t *dev, mpu6050_int_latch_t *latch)
{
    return read_reg_bits(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, BIT(MPU6050_INTCFG_LATCH_INT_EN_BIT), (uint8_t *)latch);
}

esp_err_t mpu6050_set_interrupt_latch(mpu6050_dev_t *dev, mpu6050_int_latch_t latch)
{
    CHECK_ARG(latch <= MPU6050_INT_LATCH_CONTINUOUS);

    return write_reg_bits(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, BIT(MPU6050_INTCFG_LATCH_INT_EN_BIT), latch);
}

esp_err_t mpu6050_get_interrupt_latch_clear(mpu6050_dev_t *dev, bool *clear)
{
    return read_reg_bool(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}

esp_err_t mpu6050_set_interrupt_latch_clear(mpu6050_dev_t *dev, bool clear)
{
    return write_reg_bool(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}

esp_err_t mpu6050_get_fsync_interrupt_level(mpu6050_dev_t *dev, mpu6050_int_level_t *level)
{
    return read_reg_bits(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, BIT(MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT), (uint8_t *)level);
}

esp_err_t mpu6050_set_fsync_interrupt_level(mpu6050_dev_t *dev, mpu6050_int_level_t level)
{
    CHECK_ARG(level <= MPU6050_INT_LEVEL_LOW);

    return write_reg_bits(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, BIT(MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT), level);
}

esp_err_t mpu6050_get_fsync_interrupt_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled);
}

esp_err_t mpu6050_set_fsync_interrupt_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled);
}

esp_err_t mpu6050_get_i2c_bypass_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

esp_err_t mpu6050_set_i2c_bypass_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

esp_err_t mpu6050_get_clock_output_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}

esp_err_t mpu6050_set_clock_output_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}

esp_err_t mpu6050_get_int_enabled(mpu6050_dev_t *dev, uint8_t *ints)
{
    return read_reg(dev, MPU6050_REGISTER_INT_ENABLE, ints);
}

esp_err_t mpu6050_set_int_enabled(mpu6050_dev_t *dev, uint8_t ints)
{
    return write_reg(dev, MPU6050_REGISTER_INT_ENABLE, ints);
}

esp_err_t mpu6050_get_int_status(mpu6050_dev_t *dev, uint8_t *ints)
{
    return read_reg(dev, MPU6050_REGISTER_INT_ENABLE, ints);
}

static const uint8_t accel_offs_regs[] = {
    [MPU6050_X_AXIS] = MPU6050_REGISTER_XA_OFFS_H,
    [MPU6050_Y_AXIS] = MPU6050_REGISTER_YA_OFFS_H,
    [MPU6050_Z_AXIS] = MPU6050_REGISTER_ZA_OFFS_H,
};

esp_err_t mpu6050_get_accel_offset(mpu6050_dev_t *dev, mpu6050_axis_t axis, int16_t *offset)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return read_reg_word(dev, accel_offs_regs[axis], offset);
}

esp_err_t mpu6050_set_accel_offset(mpu6050_dev_t *dev, mpu6050_axis_t axis, int16_t offset)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return write_reg_word(dev, accel_offs_regs[axis], offset);
}

static const uint8_t gyro_offs_regs[] = {
    [MPU6050_X_AXIS] = MPU6050_REGISTER_XG_OFFS_USRH,
    [MPU6050_Y_AXIS] = MPU6050_REGISTER_YG_OFFS_USRH,
    [MPU6050_Z_AXIS] = MPU6050_REGISTER_ZG_OFFS_USRH,
};

esp_err_t mpu6050_get_gyro_offset(mpu6050_dev_t *dev, mpu6050_axis_t axis, int16_t *offset)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return read_reg_word(dev, gyro_offs_regs[axis], offset);
}

esp_err_t mpu6050_set_gyro_offset(mpu6050_dev_t *dev, mpu6050_axis_t axis, int16_t offset)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return write_reg_word(dev, gyro_offs_regs[axis], offset);
}

esp_err_t mpu6050_get_raw_acceleration(mpu6050_dev_t *dev, mpu6050_raw_acceleration_t *raw_accel)
{
    CHECK_ARG(dev && raw_accel);

    uint16_t buf[3];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MPU6050_REGISTER_ACCEL_XOUT_H, buf, sizeof(buf)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    raw_accel->x = shuffle(buf[0]);
    raw_accel->y = shuffle(buf[1]);
    raw_accel->z = shuffle(buf[2]);

    return ESP_OK;
}

esp_err_t mpu6050_get_acceleration(mpu6050_dev_t *dev, mpu6050_acceleration_t *accel)
{
    CHECK_ARG(accel);

    mpu6050_raw_acceleration_t raw;
    CHECK(mpu6050_get_raw_acceleration(dev, &raw));

    accel->x = get_accel_value(dev, raw.x);
    accel->y = get_accel_value(dev, raw.y);
    accel->z = get_accel_value(dev, raw.z);

    return ESP_OK;
}

esp_err_t mpu6050_get_raw_acceleration_axis(mpu6050_dev_t *dev, mpu6050_axis_t axis, int16_t *raw_accel)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return read_reg_word(dev, MPU6050_REGISTER_ACCEL_XOUT_H + axis * 2, raw_accel);
}

esp_err_t mpu6050_get_acceleration_axis(mpu6050_dev_t *dev, mpu6050_axis_t axis, float *accel)
{
    CHECK_ARG(accel);

    int16_t raw;
    CHECK(mpu6050_get_raw_acceleration_axis(dev, axis, &raw));

    *accel = get_accel_value(dev, raw);

    return ESP_OK;
}

esp_err_t mpu6050_get_raw_rotation(mpu6050_dev_t *dev, mpu6050_raw_rotation_t *raw_gyro)
{
    CHECK_ARG(dev && raw_gyro);

    uint16_t buf[3];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MPU6050_REGISTER_GYRO_XOUT_H, buf, sizeof(buf)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    raw_gyro->x = shuffle(buf[0]);
    raw_gyro->y = shuffle(buf[1]);
    raw_gyro->z = shuffle(buf[2]);

    return ESP_OK;
}

esp_err_t mpu6050_get_rotation(mpu6050_dev_t *dev, mpu6050_rotation_t *gyro)
{
    CHECK_ARG(gyro);

    mpu6050_raw_rotation_t raw;
    CHECK(mpu6050_get_raw_rotation(dev, &raw));

    gyro->x = get_gyro_value(dev, raw.x);
    gyro->y = get_gyro_value(dev, raw.y);
    gyro->z = get_gyro_value(dev, raw.z);

    return ESP_OK;
}

esp_err_t mpu6050_get_raw_rotation_axis(mpu6050_dev_t *dev, mpu6050_axis_t axis, int16_t *raw_gyro)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return read_reg_word(dev, MPU6050_REGISTER_ACCEL_XOUT_H + axis * 2, raw_gyro);
}

esp_err_t mpu6050_get_rotation_axis(mpu6050_dev_t *dev, mpu6050_axis_t axis, float *gyro)
{
    CHECK_ARG(gyro);

    int16_t raw;
    CHECK(mpu6050_get_raw_rotation_axis(dev, axis, &raw));

    *gyro = get_gyro_value(dev, raw);

    return ESP_OK;
}

esp_err_t mpu6050_get_motion(mpu6050_dev_t *dev, mpu6050_acceleration_t *accel, mpu6050_rotation_t *gyro)
{
    CHECK(mpu6050_get_acceleration(dev, accel));
    CHECK(mpu6050_get_rotation(dev, gyro));
    return ESP_OK;
}

esp_err_t mpu6050_get_temperature(mpu6050_dev_t *dev, float *temp)
{
    CHECK_ARG(temp);

    int16_t raw = 0;
    CHECK(read_reg_word(dev, MPU6050_REGISTER_TEMP_OUT_H, &raw));
    *temp = ((float)raw / 340.0f) + 36.53f;

    return ESP_OK;
}

esp_err_t mpu6050_get_external_sensor_data(mpu6050_dev_t *dev, int position, void *buf, size_t length)
{
    CHECK_ARG(dev && buf && length && position < 24 && position + length - 1 < 24);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MPU6050_REGISTER_EXT_SENS_DATA_00 + position, buf, length));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mpu6050_get_motion_status(mpu6050_dev_t *dev, uint8_t *status)
{
    return read_reg(dev, MPU6050_REGISTER_MOT_DETECT_STATUS, status);
}

esp_err_t mpu6050_set_slave_output_byte(mpu6050_dev_t *dev, mpu6050_slave_t num, uint8_t data)
{
    CHECK_ARG(num < MPU6050_SLAVE_4);

    return write_reg(dev, MPU6050_REGISTER_I2C_SLV0_DO + num, data);
}

esp_err_t mpu6050_get_external_shadow_delay_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}

esp_err_t mpu6050_set_external_shadow_delay_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}

esp_err_t mpu6050_get_slave_delay_enabled(mpu6050_dev_t *dev, mpu6050_slave_t num, bool *enabled)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return read_reg_bool(dev, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, num, enabled);
}

esp_err_t mpu6050_set_slave_delay_enabled(mpu6050_dev_t *dev, mpu6050_slave_t num, bool enabled)
{
    CHECK_ARG(num <= MPU6050_SLAVE_4);

    return write_reg_bool(dev, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, num, enabled);
}

esp_err_t mpu6050_reset_gyroscope_path(mpu6050_dev_t *dev)
{
    return write_reg_bool(dev, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_GYRO_RESET_BIT, 1);
}

esp_err_t mpu6050_reset_accelerometer_path(mpu6050_dev_t *dev)
{
    return write_reg_bool(dev, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_ACCEL_RESET_BIT, 1);
}

esp_err_t mpu6050_reset_temperature_path(mpu6050_dev_t *dev)
{
    return write_reg_bool(dev, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_TEMP_RESET_BIT, 1);
}

esp_err_t mpu6050_get_accelerometer_power_on_delay(mpu6050_dev_t *dev, uint8_t *delay)
{
    return read_reg_bits(dev, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_DELAY_BIT, MPU6050_DETECT_ACCEL_DELAY_MASK, delay);
}

esp_err_t mpu6050_set_accelerometer_power_on_delay(mpu6050_dev_t *dev, uint8_t delay)
{
    return write_reg_bits(dev, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_DELAY_BIT, MPU6050_DETECT_ACCEL_DELAY_MASK, delay);
}

esp_err_t mpu6050_get_freefall_detection_counter_decrement(mpu6050_dev_t *dev, uint8_t *decrement)
{
    return read_reg_bits(dev, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT, MPU6050_DETECT_FF_COUNT_MASK, decrement);
}

esp_err_t mpu6050_set_freefall_detection_counter_decrement(mpu6050_dev_t *dev, uint8_t decrement)
{
    return write_reg_bits(dev, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT, MPU6050_DETECT_FF_COUNT_MASK, decrement);
}

esp_err_t mpu6050_get_motion_detection_counter_decrement(mpu6050_dev_t *dev, uint8_t *decrement)
{
    return read_reg_bits(dev, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT, MPU6050_DETECT_MOT_COUNT_MASK, decrement);
}

esp_err_t mpu6050_set_motion_detection_counter_decrement(mpu6050_dev_t *dev, uint8_t decrement)
{
    return write_reg_bits(dev, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT, MPU6050_DETECT_MOT_COUNT_MASK, decrement);
}

esp_err_t mpu6050_get_fifo_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

esp_err_t mpu6050_set_fifo_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

esp_err_t mpu6050_get_i2c_master_mode_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

esp_err_t mpu6050_set_i2c_master_mode_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

esp_err_t mpu6050_switch_spie_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled);
}

esp_err_t mpu6050_reset_fifo(mpu6050_dev_t *dev)
{
    return write_reg_bool(dev, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

esp_err_t mpu6050_reset_sensors(mpu6050_dev_t *dev)
{
    return write_reg_bool(dev, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT, 1);
}

esp_err_t mpu6050_reset(mpu6050_dev_t *dev)
{
    return write_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

esp_err_t mpu6050_get_sleep_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

esp_err_t mpu6050_set_sleep_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

esp_err_t mpu6050_get_wake_cycle_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    return read_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}

esp_err_t mpu6050_set_wake_cycle_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}

esp_err_t mpu6050_get_temp_sensor_enabled(mpu6050_dev_t *dev, bool *enabled)
{
    CHECK(read_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, enabled));
    *enabled = !*enabled;

    return ESP_OK;
}

esp_err_t mpu6050_set_temp_sensor_enabled(mpu6050_dev_t *dev, bool enabled)
{
    return write_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}

esp_err_t mpu6050_get_clock_source(mpu6050_dev_t *dev, mpu6050_clock_source_t *source)
{
    return read_reg_bits(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_MASK, (uint8_t *)source);
}

esp_err_t mpu6050_set_clock_source(mpu6050_dev_t *dev, mpu6050_clock_source_t source)
{
    return write_reg_bits(dev, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_MASK, source);
}

esp_err_t mpu6050_get_wake_frequency(mpu6050_dev_t *dev, mpu6050_wake_freq_t *frequency)
{
    return read_reg_bits(dev, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_MASK, (uint8_t *)frequency);
}

esp_err_t mpu6050_set_wake_frequency(mpu6050_dev_t *dev, mpu6050_wake_freq_t frequency)
{
    return write_reg_bits(dev, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_MASK, frequency);
}

static const uint8_t standby_accel_bits[] = {
    [MPU6050_X_AXIS] = MPU6050_PWR2_STBY_XA_BIT,
    [MPU6050_Y_AXIS] = MPU6050_PWR2_STBY_YA_BIT,
    [MPU6050_Z_AXIS] = MPU6050_PWR2_STBY_ZA_BIT,
};

esp_err_t mpu6050_get_standby_accel_enabled(mpu6050_dev_t *dev, mpu6050_axis_t axis, bool *enabled)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return read_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_2, standby_accel_bits[axis], enabled);
}

esp_err_t mpu6050_set_standby_accel_enabled(mpu6050_dev_t *dev, mpu6050_axis_t axis, bool enabled)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return write_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_2, standby_accel_bits[axis], enabled);
}

static const uint8_t standby_gyro_bits[] = {
    [MPU6050_X_AXIS] = MPU6050_PWR2_STBY_XG_BIT,
    [MPU6050_Y_AXIS] = MPU6050_PWR2_STBY_YG_BIT,
    [MPU6050_Z_AXIS] = MPU6050_PWR2_STBY_ZG_BIT,
};

esp_err_t mpu6050_get_standby_gyro_enabled(mpu6050_dev_t *dev, mpu6050_axis_t axis, bool *enabled)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return read_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_2, standby_gyro_bits[axis], enabled);
}

esp_err_t mpu6050_set_standby_gyro_enabled(mpu6050_dev_t *dev, mpu6050_axis_t axis, bool enabled)
{
    CHECK_ARG(axis <= MPU6050_Z_AXIS);

    return write_reg_bool(dev, MPU6050_REGISTER_PWR_MGMT_2, standby_gyro_bits[axis], enabled);
}

esp_err_t mpu6050_get_fifo_count(mpu6050_dev_t *dev, uint16_t *count)
{
    CHECK_ARG(dev && count);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MPU6050_REGISTER_FIFO_COUNTH, count, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *count = ushuffle(*count);
    return ESP_OK;
}

esp_err_t mpu6050_get_fifo_bytes(mpu6050_dev_t *dev, uint8_t *data, size_t length)
{
    CHECK_ARG(dev && data && length);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MPU6050_REGISTER_FIFO_COUNTH, data, length));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mpu6050_get_fifo_byte(mpu6050_dev_t *dev, uint8_t *data)
{
    return read_reg(dev, MPU6050_REGISTER_FIFO_R_W, data);
}

esp_err_t mpu6050_set_fifo_byte(mpu6050_dev_t *dev, uint8_t data)
{
    return write_reg(dev, MPU6050_REGISTER_FIFO_R_W, data);
}

esp_err_t mpu6050_get_device_id(mpu6050_dev_t *dev, uint8_t *id)
{
    return read_reg_bits(dev, MPU6050_REGISTER_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_MASK, id);
}

esp_err_t mpu6050_calibrate(mpu6050_dev_t *dev, float *accel_bias_res, float *gyro_bias_res)
{
    CHECK_ARG(dev && accel_bias_res && gyro_bias_res);

    int16_t temp_offset[3];
    int32_t accel_bias[3] = { 0, 0, 0 };
    int32_t gyro_bias[3] = { 0, 0, 0 };
    int32_t accel_bias_reg[3] = { 0, 0, 0 };
    uint16_t accel_temp[3] = { 0, 0, 0 };
    uint16_t gyro_temp[3] = { 0, 0, 0 };
    uint8_t mask_bit[3] = { 0, 0, 0 };
    uint32_t mask = 1uL;
    uint16_t gyro_sensitivity = 131;
    uint16_t accel_sensitivity = 16384;
    uint8_t tmp_data[12];
    uint16_t packet_count;

    CHECK(mpu6050_reset(dev));
    vTaskDelay(pdMS_TO_TICKS(100));

    CHECK(mpu6050_set_clock_source(dev, MPU6050_CLOCK_PLL_X));
    vTaskDelay(pdMS_TO_TICKS(200));

    // Configure device for bias calculation:
    CHECK(mpu6050_set_int_enabled(dev, false));
    CHECK(mpu6050_set_fifo_enabled(dev, false));
    CHECK(mpu6050_set_accel_fifo_enabled(dev, false));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_Z_AXIS, false));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_X_AXIS, false));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_Y_AXIS, false));
    CHECK(mpu6050_set_temp_fifo_enabled(dev, false));
    CHECK(mpu6050_set_clock_source(dev, MPU6050_CLOCK_INTERNAL));
    CHECK(mpu6050_set_multi_master_enabled(dev, false));
    CHECK(mpu6050_set_fifo_enabled(dev, false));
    CHECK(mpu6050_set_i2c_master_mode_enabled(dev, false));
    CHECK(mpu6050_reset_sensors(dev));
    vTaskDelay(pdMS_TO_TICKS(15));

    // Configure MPU6050 gyro and accelerometer for bias calculation:
    CHECK(mpu6050_set_rate(dev, 0)); // Set sample rate to 1 kHz.
    CHECK(mpu6050_set_dlpf_mode(dev, MPU6050_DLPF_1));
    CHECK(mpu6050_set_full_scale_accel_range(dev, MPU6050_ACCEL_RANGE_2));
    CHECK(mpu6050_set_full_scale_gyro_range(dev, MPU6050_GYRO_RANGE_250));

    /**
     * Configure FIFO to capture data for bias calculation.
     */

    // Enable gyroscope and accelerometer sensors for FIFO:
    CHECK(mpu6050_set_fifo_enabled(dev, true));
    CHECK(mpu6050_set_accel_fifo_enabled(dev, true));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_Z_AXIS, true));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_X_AXIS, true));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_Y_AXIS, true));
    vTaskDelay(pdMS_TO_TICKS(80)); // Accumulate 80 samples in 80 ms.

    // At end of sample accumulation, turn off FIFO sensor read:
    CHECK(mpu6050_set_fifo_enabled(dev, false));
    CHECK(mpu6050_set_accel_fifo_enabled(dev, false));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_Z_AXIS, false));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_X_AXIS, false));
    CHECK(mpu6050_set_gyro_fifo_enabled(dev, MPU6050_Y_AXIS, false));
    CHECK(mpu6050_set_temp_fifo_enabled(dev, false));

    // Sets of full gyro and accelerometer data for averaging:
    CHECK(mpu6050_get_fifo_count(dev, &packet_count));
    packet_count /= 12;

    for (int i = 0; i < packet_count; i++)
    {
        // Read data for averaging:
        CHECK(mpu6050_get_fifo_bytes(dev, &tmp_data[0], 6));
        accel_temp[0] = (int16_t)(((int16_t)tmp_data[0] << 8) | tmp_data[1]);
        accel_temp[1] = (int16_t)(((int16_t)tmp_data[2] << 8) | tmp_data[3]);
        accel_temp[2] = (int16_t)(((int16_t)tmp_data[4] << 8) | tmp_data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)tmp_data[6] << 8) | tmp_data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)tmp_data[8] << 8) | tmp_data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)tmp_data[10] << 8) | tmp_data[11]);

        // Sum individual 16-bit biases to get accumulated signed 32-bit biases:
        accel_bias[0] += (int32_t)accel_temp[0];
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }

    // Normalize sums to get average count biases:
    accel_bias[0] /= (int32_t)packet_count;
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    // Remove gravity from the z-axis accelerometer bias calculation:
    if (accel_bias[2] > 0L)
        accel_bias[2] -= (int32_t)accel_sensitivity;
    else
        accel_bias[2] += (int32_t)accel_sensitivity;

    /**
     * Construct the gyro biases for push to the hardware gyro bias registers,
     * which are reset to zero upon device startup:
     */

    // Divide by 4 to get 32.9 LSB per deg/s to expected bias input format.
    // Biases are additive, so change sign on calculated average gyro biases.
    tmp_data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
    tmp_data[1] = (-gyro_bias[0] / 4) & 0xFF;
    tmp_data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    tmp_data[3] = (-gyro_bias[1] / 4) & 0xFF;
    tmp_data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    tmp_data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers:
    CHECK(mpu6050_set_gyro_offset(dev, MPU6050_X_AXIS, ((int16_t)tmp_data[0]) << 8 | tmp_data[1]));
    CHECK(mpu6050_set_gyro_offset(dev, MPU6050_Y_AXIS, ((int16_t)tmp_data[2]) << 8 | tmp_data[3]));
    CHECK(mpu6050_set_gyro_offset(dev, MPU6050_Z_AXIS, ((int16_t)tmp_data[4]) << 8 | tmp_data[5]));

    // Construct gyro bias in deg/s for later manual subtraction:
    gyro_bias_res[0] = (float)gyro_bias[0] / (float)gyro_sensitivity;
    gyro_bias_res[1] = (float)gyro_bias[1] / (float)gyro_sensitivity;
    gyro_bias_res[2] = (float)gyro_bias[2] / (float)gyro_sensitivity;

    /**
     * Construct the accelerometer biases for push to the hardware accelerometer
     * bias registers. These registers contain factory trim values which must be
     * added to the calculated accelerometer biases; on boot up these registers
     * will hold non-zero values. In addition, bit 0 of the lower byte must be
     * preserved since it is used for temperature compensation calculations.
     * Accelerometer bias registers expect bias input as 2048 LSB per g, so that
     * the accelerometer biases calculated above must be divided by 8.
     */

    // Read factory accelerometer trim values:
    CHECK(mpu6050_get_accel_offset(dev, MPU6050_X_AXIS, &(temp_offset[0])));
    CHECK(mpu6050_get_accel_offset(dev, MPU6050_Y_AXIS, &(temp_offset[1])));
    CHECK(mpu6050_get_accel_offset(dev, MPU6050_Z_AXIS, &(temp_offset[2])));

    for (int i = 0; i < 3; i++)
    {
        // If temperature compensation bit is set, record that in mask_bit:
        if (accel_bias_reg[i] & mask)
            mask_bit[i] = 0x01;
    }

    /**
     * Construct total accelerometer bias, including calculated average
     * accelerometer bias from above (Subtract calculated averaged accelerometer
     * bias scaled to 2048 LSB/g (16g full scale).
     */

    accel_bias_reg[0] -= (accel_bias[0] / 8);
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    tmp_data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    tmp_data[1] = (accel_bias_reg[0]) & 0xFF;
    tmp_data[1] = tmp_data[1] | mask_bit[0];
    tmp_data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    tmp_data[3] = (accel_bias_reg[1]) & 0xFF;
    tmp_data[3] = tmp_data[3] | mask_bit[1];
    tmp_data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    tmp_data[5] = (accel_bias_reg[2]) & 0xFF;
    tmp_data[5] = tmp_data[5] | mask_bit[2];

    // Push accelerometer biases to hardware registers:
    CHECK(mpu6050_set_accel_offset(dev, MPU6050_X_AXIS, ((int16_t)tmp_data[0]) << 8 | tmp_data[1]));
    CHECK(mpu6050_set_accel_offset(dev, MPU6050_X_AXIS, ((int16_t)tmp_data[2]) << 8 | tmp_data[3]));
    CHECK(mpu6050_set_accel_offset(dev, MPU6050_X_AXIS, ((int16_t)tmp_data[4]) << 8 | tmp_data[5]));

    // Output scaled accelerometer biases for subtraction in the main program:
    accel_bias_res[0] = (float)accel_bias[0] / (float)accel_sensitivity;
    accel_bias_res[1] = (float)accel_bias[1] / (float)accel_sensitivity;
    accel_bias_res[2] = (float)accel_bias[2] / (float)accel_sensitivity;

    return ESP_OK;
}

esp_err_t mpu6050_self_test(mpu6050_dev_t *dev, float *destination)
{
    uint8_t self_test[6];
    float factory_trim[6];

    // Configure the accelerometer for self-test:
    CHECK(mpu6050_set_accel_self_test(dev, MPU6050_X_AXIS, true));
    CHECK(mpu6050_set_accel_self_test(dev, MPU6050_Y_AXIS, true));
    CHECK(mpu6050_set_accel_self_test(dev, MPU6050_Z_AXIS, true));
    CHECK(mpu6050_set_full_scale_accel_range(dev, MPU6050_ACCEL_RANGE_8));
    CHECK(mpu6050_set_full_scale_gyro_range(dev, MPU6050_GYRO_RANGE_250));

    CHECK(mpu6050_get_accel_self_test_factory_trim(dev, MPU6050_X_AXIS, &self_test[0]));
    CHECK(mpu6050_get_accel_self_test_factory_trim(dev, MPU6050_Y_AXIS, &self_test[1]));
    CHECK(mpu6050_get_accel_self_test_factory_trim(dev, MPU6050_Z_AXIS, &self_test[2]));
    CHECK(mpu6050_get_gyro_self_test_factory_trim(dev, MPU6050_X_AXIS, &self_test[3]));
    CHECK(mpu6050_get_gyro_self_test_factory_trim(dev, MPU6050_Y_AXIS, &self_test[4]));
    CHECK(mpu6050_get_gyro_self_test_factory_trim(dev, MPU6050_Z_AXIS, &self_test[5]));

    // Process results to allow final comparison with factory set values:
    factory_trim[0] = (4096.0f * 0.34f) * (powf((0.92f / 0.34f), (((float)self_test[0] - 1.0f) / 30.0f)));
    factory_trim[1] = (4096.0f * 0.34f) * (powf((0.92f / 0.34f), (((float)self_test[1] - 1.0f) / 30.0f)));
    factory_trim[2] = (4096.0f * 0.34f) * (powf((0.92f / 0.34f), (((float)self_test[2] - 1.0f) / 30.0f)));
    factory_trim[3] = (25.0f * 131.0f) * (powf(1.046f, ((float)self_test[3] - 1.0f)));
    factory_trim[4] = (-25.0f * 131.0f) * (powf(1.046f, ((float)self_test[4] - 1.0f)));
    factory_trim[5] = (25.0f * 131.0f) * (powf(1.046f, ((float)self_test[5] - 1.0f)));

    // Report results as a ratio of "(STR - FT) / FT" (The change from Factory Trim of the Self-Test Response).
    // To get to percent, must multiply by 100 and subtract result from 100.
    for (int i = 0; i < 6; i++)
        destination[i] = 100.0f + 100.0f * ((float)self_test[i] - factory_trim[i]) / factory_trim[i];
    return ESP_OK;
}

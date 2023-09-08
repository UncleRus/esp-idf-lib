/*
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file pca9685.c
 *
 * ESP-IDF driver for 16-channel, 12-bit PWM PCA9685
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include "pca9685.h"
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>
#include <esp_system.h>
#include <esp_log.h>
#include <ets_sys.h>

#define I2C_FREQ_HZ 1000000 // 1 Mhz

#define REG_MODE1      0x00
#define REG_MODE2      0x01
#define REG_SUBADR1    0x02
#define REG_SUBADR2    0x03
#define REG_SUBADR3    0x04
#define REG_ALLCALLADR 0x05
#define REG_LEDX       0x06
#define REG_ALL_LED    0xfa
#define REG_PRE_SCALE  0xfe

#define MODE1_RESTART (1 << 7)
#define MODE1_EXTCLK  (1 << 6)
#define MODE1_AI      (1 << 5)
#define MODE1_SLEEP   (1 << 4)

#define MODE1_SUB_BIT 3

#define MODE2_INVRT   (1 << 4)
#define MODE2_OUTDRV  (1 << 2)

#define LED_FULL_ON_OFF (1 << 4)

#define REG_LED_N(x)  (REG_LEDX + (x) * 4)
#define OFFS_REG_LED_ON  1
#define OFFS_REG_LED_OFF 3

#define INTERNAL_FREQ 25000000

#define MIN_PRESCALER 0x03
#define MAX_PRESCALER 0xff
#define MAX_SUBADDR   2

#define WAKEUP_DELAY_US 500

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_ARG_LOGE(VAL, msg, ...) do { if (!(VAL)) { ESP_LOGE(TAG, msg, ## __VA_ARGS__); return ESP_ERR_INVALID_ARG; } } while (0)
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)

static const char *TAG = "pca9685";

inline static uint32_t round_div(uint32_t x, uint32_t y)
{
    return (x + y / 2) / y;
}

inline static esp_err_t write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(dev, reg, &val, 1);
}

inline static esp_err_t read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(dev, reg, val, 1);
}

static esp_err_t update_reg(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t v;

    CHECK(read_reg(dev, reg, &v));
    v = (v & ~mask) | val;
    CHECK(write_reg(dev, reg, v));

    return ESP_OK;
}

static esp_err_t dev_sleep(i2c_dev_t *dev, bool sleep)
{
    CHECK(update_reg(dev, REG_MODE1, MODE1_SLEEP, sleep ? MODE1_SLEEP : 0));
    if (!sleep)
        ets_delay_us(WAKEUP_DELAY_US);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// Public

esp_err_t pca9685_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t pca9685_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t pca9685_init(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    // Enable autoincrement
    I2C_DEV_CHECK(dev, update_reg(dev, REG_MODE1, MODE1_AI, MODE1_AI));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_set_subaddr(i2c_dev_t *dev, uint8_t num, uint8_t subaddr, bool enable)
{
    CHECK_ARG(dev);
    CHECK_ARG_LOGE(num <= MAX_SUBADDR, "Invalid subadress number (%d), must be in (0..2)", num);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg(dev, REG_SUBADR1 + num, subaddr << 1));

    uint8_t mask = 1 << (MODE1_SUB_BIT - num);
    I2C_DEV_CHECK(dev, update_reg(dev, REG_MODE1, mask, enable ? mask : 0));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_restart(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    uint8_t mode;
    I2C_DEV_CHECK(dev, read_reg(dev, REG_MODE1, &mode));
    if (mode & MODE1_RESTART)
    {
        I2C_DEV_CHECK(dev, write_reg(dev, REG_MODE1, mode & ~MODE1_SLEEP));
        ets_delay_us(WAKEUP_DELAY_US);
    }
    I2C_DEV_CHECK(dev, write_reg(dev, REG_MODE1, (mode & ~MODE1_SLEEP) | MODE1_RESTART));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_is_sleeping(i2c_dev_t *dev, bool *sleeping)
{
    CHECK_ARG(dev && sleeping);

    uint8_t v;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_MODE1, &v));
    I2C_DEV_GIVE_MUTEX(dev);
    *sleeping = v & MODE1_SLEEP;

    return ESP_OK;
}

esp_err_t pca9685_sleep(i2c_dev_t *dev, bool sleep)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, dev_sleep(dev, sleep));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_is_output_inverted(i2c_dev_t *dev, bool *inv)
{
    CHECK_ARG(dev && inv);

    uint8_t v;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_MODE2, &v));
    I2C_DEV_GIVE_MUTEX(dev);
    *inv = v & MODE2_INVRT;

    return ESP_OK;
}

esp_err_t pca9685_set_output_inverted(i2c_dev_t *dev, bool inverted)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_reg(dev, REG_MODE2, MODE2_INVRT, inverted ? MODE2_INVRT : 0));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_get_output_open_drain(i2c_dev_t *dev, bool *od)
{
    CHECK_ARG(dev && od);

    uint8_t v;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_MODE2, &v));
    I2C_DEV_GIVE_MUTEX(dev);
    *od = v & MODE2_OUTDRV;

    return ESP_OK;
}

esp_err_t pca9685_set_output_open_drain(i2c_dev_t *dev, bool od)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_reg(dev, REG_MODE2, MODE2_OUTDRV, od ? 0 : MODE2_OUTDRV));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_get_prescaler(i2c_dev_t *dev, uint8_t *prescaler)
{
    CHECK_ARG(dev && prescaler);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_PRE_SCALE, prescaler));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_set_prescaler(i2c_dev_t *dev, uint8_t prescaler)
{
    CHECK_ARG(dev);
    CHECK_ARG_LOGE(prescaler >= MIN_PRESCALER,
            "Invalid prescaler value: (%" PRIu8 "), must be >= 3", prescaler);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, dev_sleep(dev, true));
    I2C_DEV_CHECK(dev, write_reg(dev, REG_PRE_SCALE, prescaler));
    I2C_DEV_CHECK(dev, dev_sleep(dev, false));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_get_pwm_frequency(i2c_dev_t *dev, uint16_t *freq)
{
    CHECK_ARG(dev);
    CHECK_ARG(freq);

    uint8_t prescale;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg(dev, REG_PRE_SCALE, &prescale));
    I2C_DEV_GIVE_MUTEX(dev);
    *freq = INTERNAL_FREQ / ((uint32_t)PCA9685_MAX_PWM_VALUE * (prescale + 1));

    return ESP_OK;
}

esp_err_t pca9685_set_pwm_frequency(i2c_dev_t *dev, uint16_t freq)
{
    uint32_t prescaler = round_div(INTERNAL_FREQ, (uint32_t)PCA9685_MAX_PWM_VALUE * freq) - 1;
    CHECK_ARG_LOGE(prescaler >= MIN_PRESCALER && prescaler <= MAX_PRESCALER,
            "Invalid prescaler value (%" PRIu32 "), must be in (%d..%d)", prescaler,
            MIN_PRESCALER, MAX_PRESCALER);
    return pca9685_set_prescaler(dev, prescaler);
}

esp_err_t pca9685_set_pwm_value(i2c_dev_t *dev, uint8_t channel, uint16_t val)
{
    CHECK_ARG(dev);
    CHECK_ARG_LOGE(channel <= PCA9685_CHANNEL_ALL,
            "Invalid channel %d, must be in (0..%d)", channel, PCA9685_CHANNEL_ALL);
    CHECK_ARG_LOGE(val <= PCA9685_MAX_PWM_VALUE,
            "Invalid PWM value %d, must be in (0..PCA9685_MAX_PWM_VALUE)", val);

    uint8_t reg = channel == PCA9685_CHANNEL_ALL ? REG_ALL_LED : REG_LED_N(channel);

    bool full_on = val >= PCA9685_MAX_PWM_VALUE;
    bool full_off = val == 0;

    uint16_t raw = full_on ? 4095 : val;

    uint8_t buf[4] = {
        0,
        full_on ? LED_FULL_ON_OFF : 0,
        raw,
        full_off ? LED_FULL_ON_OFF | (raw >> 8) : raw >> 8
    };

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, buf, 4));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t pca9685_set_pwm_values(i2c_dev_t *dev, uint8_t first_ch, uint8_t channels,
        const uint16_t *values)
{
    CHECK_ARG(values);
    CHECK_ARG_LOGE(channels > 0 && first_ch + channels - 1 < PCA9685_CHANNEL_ALL,
            "Invalid first_ch or channels: (%d, %d)", first_ch, channels);


    size_t size = channels * 4;
    uint8_t buf[size];
    for (uint8_t ch = first_ch; ch < first_ch + channels; ch++)
    {
        bool full_on = values[ch] >= PCA9685_MAX_PWM_VALUE;
        bool full_off = values[ch] == 0;

        uint16_t val = full_on ? 4095 : values[ch];

        buf[ch * 4] = 0;
        buf[ch * 4 + 1] = full_on ? LED_FULL_ON_OFF : 0;
        buf[ch * 4 + 2] = val;
        buf[ch * 4 + 3] = full_off ? LED_FULL_ON_OFF | (val >> 8) : val >> 8;
    }

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_LED_N(first_ch), buf, size));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

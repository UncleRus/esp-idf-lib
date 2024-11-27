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
 * @file pca9632.c
 * @defgroup pca9632 pca9632
 * @{
 *
 * ESP-IDF Driver for PCA9632 4-channel PWM chip
 *
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_idf_lib_helpers.h>
#include "pca9632.h"

#define I2C_FREQ_HZ 400000

// Register definitions (page 11, table 7)
#define REG_MODE1      0x00
#define REG_MODE2      0x01
#define REG_PWM0       0x02
#define REG_PWM1       0x03
#define REG_PWM2       0x04
#define REG_PWM3       0x05
#define REG_GRPPWM     0x06
#define REG_GRPFREQ    0x07
#define REG_LEDOUT     0x08
#define REG_SUBADR1    0x09
#define REG_SUBADR2    0x0A
#define REG_SUBADR3    0x0B
#define REG_ALLCALLADR 0x0C

// Bits in REG_MODE1 (page 12, table 8)
#define BIT_AI2     7
#define BIT_AI1     6
#define BIT_AI0     5
#define BIT_SLEEP   4
#define BIT_SUB1    3
#define BIT_SUB2    2
#define BIT_SUB3    1
#define BIT_ALLCALL 0

// Bits in REG_MODE2 (page 12-13, table 9)
#define BIT_DMBLNK 5
#define BIT_INVRT  4
#define BIT_OCH    3
#define BIT_OUTDRV 2
#define BIT_OUTNE1 1
#define BIT_OUTNE0 0

// Bits in REG_LEDOUT (page 14, table 13)
#define BIT_LDR3 6
#define BIT_LDR2 4
#define BIT_LDR1 2
#define BIT_LDR0 0

#define CHECK(x)                                                                                                                                                                                       \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t __;                                                                                                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                                                                                                        \
            return __;                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    while (0)
#define CHECK_ARG(VAL)                                                                                                                                                                                 \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (!(VAL))                                                                                                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                                                                                                \
    }                                                                                                                                                                                                  \
    while (0)

static esp_err_t pca9632_read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t pca9632_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &val, 1));
    I2C_DEV_GIVE_MUTEX(dev);
    return ESP_OK;
}

esp_err_t pca9632_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev && (addr == PCA9632_I2C_ADDR));

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t pca9632_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(dev);
}

esp_err_t pca9632_init(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    // clear/ reset registers
    pca9632_write_reg(dev, REG_MODE1, 0x00);
    pca9632_write_reg(dev, REG_MODE2, 0x00);
    // set default states
    pca9632_set_led_driver_all(dev, LDR_INDIV);
    pca9632_set_group_control_mode(dev, GROUP_CONTROL_MODE_DIMMING);
    return ESP_OK;
}

esp_err_t pca9632_set_autoincrement(i2c_dev_t *dev, pca9632_autoincr_mode_t ai)
{
    CHECK_ARG(dev);
    uint8_t val;

    switch (ai)
    {
        case AI_ALL:
            val = (1 << BIT_AI2) | (0 << BIT_AI1) | (0 << BIT_AI0);
            break;

        case AI_INDIV:
            val = (1 << BIT_AI2) | (1 << BIT_AI1) | (0 << BIT_AI0);
            break;

        case AI_GLOBAL:
            val = (1 << BIT_AI2) | (0 << BIT_AI1) | (1 << BIT_AI0);
            break;

        case AI_INDIV_GLOBAL:
            val = (1 << BIT_AI2) | (1 << BIT_AI1) | (1 << BIT_AI0);
            break;

        case AI_DISABLED:
        default:
            val = (0 << BIT_AI2) | (0 << BIT_AI1) | (0 << BIT_AI0);
            break;
    }

    return pca9632_write_reg(dev, REG_MODE1, val);
}

esp_err_t pca9632_set_group_control_mode(i2c_dev_t *dev, pca9632_gcm_t mode)
{
    CHECK_ARG(dev);
    uint8_t reg;

    pca9632_read_reg(dev, REG_MODE2, &reg);
    switch (mode)
    {
        case GROUP_CONTROL_MODE_BLINKING:
            return pca9632_write_reg(dev, REG_MODE2, reg | (1 << BIT_DMBLNK));

        case GROUP_CONTROL_MODE_DIMMING:
        default:
            return pca9632_write_reg(dev, REG_MODE2, reg & ~(1 << BIT_DMBLNK));
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t pca9632_set_output_params(i2c_dev_t *dev, bool invert, pca9632_outdrv_t outdrv)
{
    CHECK_ARG(dev);
    uint8_t reg;
    uint8_t val;

    pca9632_read_reg(dev, REG_MODE2, &reg);

    val = reg & ~((1 << BIT_INVRT) | (1 << BIT_OUTDRV));          // clear bits
    val = reg | ((invert << BIT_INVRT) | (outdrv << BIT_OUTDRV)); // set bits
    return pca9632_write_reg(dev, REG_MODE2, val);
}

esp_err_t pca9632_set_pwm(i2c_dev_t *dev, pca9632_led_t channel, uint8_t duty)
{
    CHECK_ARG(dev);

    switch (channel)
    {
        case LED0:
            return pca9632_write_reg(dev, REG_PWM0, duty);
        case LED1:
            return pca9632_write_reg(dev, REG_PWM1, duty);
        case LED2:
            return pca9632_write_reg(dev, REG_PWM2, duty);
        case LED3:
            return pca9632_write_reg(dev, REG_PWM3, duty);
        default:
            break;
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t pca9632_set_pwm_all(i2c_dev_t *dev, uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3)
{
    CHECK_ARG(dev);

    CHECK(pca9632_write_reg(dev, REG_PWM0, led0));
    CHECK(pca9632_write_reg(dev, REG_PWM1, led1));
    CHECK(pca9632_write_reg(dev, REG_PWM2, led2));
    CHECK(pca9632_write_reg(dev, REG_PWM3, led3));
    return ESP_ERR_INVALID_ARG;
}

esp_err_t pca9632_set_grp_pwm(i2c_dev_t *dev, uint8_t val)
{
    CHECK_ARG(dev);
    return pca9632_write_reg(dev, REG_GRPPWM, val);
}

esp_err_t pca9632_set_grp_freq(i2c_dev_t *dev, uint8_t val)
{
    CHECK_ARG(dev);
    return pca9632_write_reg(dev, REG_GRPFREQ, val);
}

esp_err_t pca9632_set_led_driver(i2c_dev_t *dev, pca9632_led_t channel, pca9632_ldr_t ldr)
{
    CHECK_ARG(dev);
    uint8_t reg;
    uint8_t val;
    uint8_t shift;

    pca9632_read_reg(dev, REG_LEDOUT, &reg);
    switch (channel)
    {
        case LED0:
            shift = BIT_LDR0;
            break;
        case LED1:
            shift = BIT_LDR1;
            break;
        case LED2:
            shift = BIT_LDR2;
            break;
        case LED3:
            shift = BIT_LDR3;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    val = reg & ~(0b11 << shift); // clear both bits of LDR
    val |= (ldr << shift);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t pca9632_set_led_driver_all(i2c_dev_t *dev, pca9632_ldr_t ldr)
{
    CHECK_ARG(dev);
    uint8_t val = (ldr << BIT_LDR3 | ldr << BIT_LDR2 | ldr << BIT_LDR1 | ldr << BIT_LDR0);
    return pca9632_write_reg(dev, REG_LEDOUT, val);
}

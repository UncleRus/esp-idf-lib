/*
 * Copyright (c) 2021 saasaa <mail@saasaa.xyz>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file hts221.c
 * @brief HTS221 driver
 * @author saasaa
 *
 * ESP-IDF driver for HTS221 temperature and humidity sensor
 *
 * Datasheet: http://www.st.com/resource/en/datasheet/hts221.pdf
 * Technical note on interpreting humidity and temperature readings in the
 * HTS221 digital humidity sensor:
 * https://www.st.com/resource/en/technical_note/tn1218-interpreting-humidity-and-temperature-readings-in-the-hts221-digital-humidity-sensor-stmicroelectronics.pdf
 * Copyright (c) 2021 saasaa <mail@saasaa.xyz>
 *
 * ISC Licensed as described in the file LICENSE
 *
 */
#include "hts221.h"

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2cdev.h>
#include <esp_timer.h>

#define I2C_FREQ_HZ 400000 // 400kHz

#define REG_WHOAMI 0x0F         // R   Device identification register
#define REG_AV_CONF 0x10        // R/W Humidity and temperature resolution mode
#define REG_CTRL_REG1 0x20      // R/W Control register 1
#define REG_CTRL_REG2 0x21      // R/W Control register 2
#define REG_CTRL_REG3 0x22      // R/W Control register 3 for data ready output signal
#define REG_STATUS_REG 0x27     // R   Status register
#define REG_HUMIDITY_OUT_L 0x28 // R   Relative humidity output register (LSB)
#define REG_HUMIDITY_OUT_H 0x29 // R   Relative humidity output register (MSB)
#define REG_TEMP_OUT_L 0x2A     // R   Temperature output register (LSB)
#define REG_TEMP_OUT_H 0x2B     // R   Temperature output register (MSB)
#define REG_CALIB_START 0x30    // R/W Calibration start register

#define BIT_AV_CONF_AVGT0      3
#define BIT_AV_CONF_AVGH0      0

#define BIT_CTRL_REG1_PD       7
#define BIT_CTRL_REG1_BDU      2
#define BIT_CTRL_REG1_ODR      0

#define BIT_CTRL_REG2_BOOT     7
#define BIT_CTRL_REG2_HEATER   1
#define BIT_CTRL_REG2_ONE_SHOT 7

#define BIT_CTRL_REG3_DRDY_H_L 7
#define BIT_CTRL_REG3_PP_OD    6
#define BIT_CTRL_REG3_DRDY     2

#define BIT_STATUS_REG_H_DA    1
#define BIT_STATUS_REG_T_DA    0

#define MASK_AV_CONF_AVG   0x07
#define MASK_CTRL_REG1_ODR 0x03
#define MASK_STATUS_REG_DA 0x03

#define WHOAMI 0xbc
#define REBOOT_TIMEOUT_US (100 * 1000)

#define HTS221_AV_CONF_DEFAULT 0x1B // DEFAULT AV_CONF status register value according to datasheet

#define CHECK(x)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        esp_err_t __;                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                        \
            return __;                                                                                                 \
    } while (0)

#define CHECK_ARG(VAL)                                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(VAL))                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                \
    } while (0)

static const char *TAG = "hts221";

static esp_err_t write_reg(hts221_t *dev, uint8_t reg, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, &val, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_reg(hts221_t *dev, uint8_t reg, uint8_t *val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, val, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t update_reg(hts221_t *dev, uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t b;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, &b, 1));
    b = (b & mask) | val;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg, &b, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

inline static esp_err_t read_calibration_coeff(hts221_t *dev)
{
    uint8_t T0_T1_msb;
    uint8_t H0_rH_x2;
    uint8_t H1_rH_x2;
    uint8_t T0_degC_x8_lsb;
    uint8_t T1_degC_x8_lsb;
    uint8_t H0_T0_OUT_lsb;
    uint8_t H0_T0_OUT_msb;
    uint8_t H1_T0_OUT_lsb;
    uint8_t H1_T0_OUT_msb;
    uint8_t T0_OUT_lsb;
    uint8_t T0_OUT_msb;
    uint8_t T1_OUT_lsb;
    uint8_t T1_OUT_msb;

    // TODO: replace with block reading
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x30, &H0_rH_x2, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x31, &H1_rH_x2, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x32, &T0_degC_x8_lsb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x33, &T1_degC_x8_lsb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x35, &T0_T1_msb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x36, &H0_T0_OUT_lsb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x37, &H0_T0_OUT_msb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x3A, &H1_T0_OUT_lsb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x3B, &H1_T0_OUT_msb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x3C, &T0_OUT_lsb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x3D, &T0_OUT_msb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x3E, &T1_OUT_lsb, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, 0x3F, &T1_OUT_msb, 1));

    dev->cal.H0_rH = (H0_rH_x2 >> 1);
    dev->cal.H1_rH = (H1_rH_x2 >> 1);
    dev->cal.T0_degC = ((T0_T1_msb & 0x03) << 8 | T0_degC_x8_lsb) >> 3;
    dev->cal.T1_degC = ((T0_T1_msb & 0x0C) << 6 | T1_degC_x8_lsb) >> 3;
    dev->cal.H0_T0_OUT = (H0_T0_OUT_msb << 8) | H0_T0_OUT_lsb;
    dev->cal.H1_T0_OUT = (H1_T0_OUT_msb << 8) | H1_T0_OUT_lsb;
    dev->cal.T0_OUT = (T0_OUT_msb << 8) | T0_OUT_lsb;
    dev->cal.T1_OUT = (T1_OUT_msb << 8) | T1_OUT_lsb;

    ESP_LOGD(TAG, "Read calibration data");

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t hts221_init_desc(hts221_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = HTS221_I2C_ADDRESS;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t hts221_free_desc(hts221_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t hts221_init(hts221_t *dev)
{
    CHECK_ARG(dev);

    esp_err_t res = ESP_OK;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t b;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_WHOAMI, &b, 1));
    if (b != WHOAMI)
    {
        ESP_LOGE(TAG, "Invalid WHOAMI value: 0x%02x", b);
        res = ESP_ERR_INVALID_RESPONSE;
        goto exit;
    }

    // Reboot
    b = BIT(BIT_CTRL_REG2_BOOT);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, REG_CTRL_REG2, &b, 1));
    // Wait
    int64_t start = esp_timer_get_time();
    while (true)
    {
        if ((esp_timer_get_time() - start) >= REBOOT_TIMEOUT_US)
        {
            ESP_LOGE(TAG, "Timeout while rebooting device");
            res = ESP_ERR_TIMEOUT;
            goto exit;
        }
        I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_CTRL_REG2, &b, 1));
        if (!(b & BIT(BIT_CTRL_REG2_BOOT)))
            break;
        vTaskDelay(1);
    }

    // Read calibration data
    I2C_DEV_CHECK(&dev->i2c_dev, read_calibration_coeff(dev));

    // Enable BDU, switch to active mode, DATA RATE = one shot
    b = BIT(BIT_CTRL_REG1_PD) | BIT(BIT_CTRL_REG1_BDU);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, REG_CTRL_REG2, &b, 1));

    // DRDY mode = Push-pull, DRDY active level = HIGH
    b = 0;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, REG_CTRL_REG3, &b, 1));

exit:
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return res;
}

esp_err_t hts221_get_power_mode(hts221_t *dev, bool *power_down)
{
    CHECK_ARG(dev && power_down);

    uint8_t raw;
    CHECK(read_reg(dev, REG_CTRL_REG1, &raw));
    *power_down = (raw & BIT(BIT_CTRL_REG1_PD)) ? true : false;

    return ESP_OK;
}

esp_err_t hts221_set_power_mode(hts221_t *dev, bool power_down)
{
    CHECK_ARG(dev);

    return update_reg(dev, REG_CTRL_REG1, power_down ? BIT(BIT_CTRL_REG1_PD) : 0, BIT(BIT_CTRL_REG1_PD));
}


esp_err_t hts221_get_data_rate(hts221_t *dev, hts221_data_rate_t *dr)
{
    CHECK_ARG(dev && dr);

    uint8_t raw;
    CHECK(read_reg(dev, REG_CTRL_REG1, &raw));
    *dr = (raw & MASK_CTRL_REG1_ODR);

    return ESP_OK;
}

esp_err_t hts221_set_data_rate(hts221_t *dev, hts221_data_rate_t dr)
{
    CHECK_ARG(dev && dr <= HTS221_12_5HZ);

    return update_reg(dev, REG_CTRL_REG1, dr, MASK_CTRL_REG1_ODR);
}

esp_err_t hts221_get_heater(hts221_t *dev, bool *enable)
{
    CHECK_ARG(dev && enable);

    uint8_t raw;
    CHECK(read_reg(dev, REG_CTRL_REG2, &raw));
    *enable = (raw & BIT(BIT_CTRL_REG2_HEATER)) ? true : false;

    return ESP_OK;
}

esp_err_t hts221_set_heater(hts221_t *dev, bool enable)
{
    CHECK_ARG(dev);

    return update_reg(dev, REG_CTRL_REG2, enable ? BIT(BIT_CTRL_REG2_HEATER) : 0, BIT(BIT_CTRL_REG2_HEATER));
}

esp_err_t hts221_get_averaging(hts221_t *dev, hts221_temperature_avg_t *t_avg, hts221_humidity_avg_t *rh_avg)
{
    CHECK_ARG(dev && (t_avg || rh_avg));

    uint8_t raw;
    CHECK(read_reg(dev, REG_AV_CONF, &raw));
    if (t_avg)
        *t_avg = (raw >> BIT_AV_CONF_AVGT0) & MASK_AV_CONF_AVG;
    if (rh_avg)
        *t_avg = (raw >> BIT_AV_CONF_AVGH0) & MASK_AV_CONF_AVG;

    return ESP_OK;
}

esp_err_t hts221_set_averaging(hts221_t *dev, hts221_temperature_avg_t t_avg, hts221_humidity_avg_t rh_avg)
{
    CHECK_ARG(dev && t_avg <= HTS221_AVGT_256 && rh_avg <= HTS221_AVGH_512);

    return write_reg(dev, REG_AV_CONF, (t_avg << BIT_AV_CONF_AVGT0) | (rh_avg << BIT_AV_CONF_AVGH0));
}

esp_err_t hts221_get_drdy_config(hts221_t *dev, bool *enable, hts221_drdy_mode_t *mode, hts221_drdy_level_t *active)
{
    CHECK_ARG(dev && (enable || mode || active));

    uint8_t raw;
    CHECK(read_reg(dev, REG_CTRL_REG3, &raw));

    if (enable)
        *enable = (raw >> BIT_CTRL_REG3_DRDY) & 1;
    if (mode)
        *mode = (raw >> BIT_CTRL_REG3_PP_OD) & 1;
    if (active)
        *active = (raw >> BIT_CTRL_REG3_DRDY_H_L) & 1;

    return ESP_OK;
}

esp_err_t hts221_set_drdy_config(hts221_t *dev, bool enable, hts221_drdy_mode_t mode, hts221_drdy_level_t active)
{
    CHECK_ARG(dev && mode <= HTS221_DRDY_OPEN_DRAIN && active <= HTS221_DRDY_ACTIVE_LOW);

    return write_reg(dev, REG_CTRL_REG3,
            (enable ? BIT(BIT_CTRL_REG3_DRDY) : 0) | (mode << BIT_CTRL_REG3_PP_OD) | (active << BIT_CTRL_REG3_DRDY_H_L));
}

esp_err_t hts221_is_data_ready(hts221_t *dev, bool *ready)
{
    CHECK_ARG(dev && ready);

    uint8_t raw;
    CHECK(read_reg(dev, REG_STATUS_REG, &raw));

    *ready = (raw & MASK_STATUS_REG_DA) == MASK_STATUS_REG_DA;

    return ESP_OK;
}

esp_err_t hts221_start_oneshot(hts221_t *dev)
{
    CHECK_ARG(dev);

    return update_reg(dev, REG_CTRL_REG2, BIT(BIT_CTRL_REG2_ONE_SHOT), BIT(BIT_CTRL_REG2_ONE_SHOT));
}

esp_err_t hts221_get_data(hts221_t *dev, float *t, float *rh)
{
    CHECK_ARG(dev && (t || rh));

    int16_t raw_rh;
    int16_t raw_t;
    uint8_t raw[4];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_HUMIDITY_OUT_L, &raw[0], 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_HUMIDITY_OUT_H, &raw[1], 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_TEMP_OUT_L, &raw[2], 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_TEMP_OUT_H, &raw[3], 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    raw_rh = (raw[1] << 8) | raw[0];
    raw_t  = (raw[3] << 8) | raw[2];

    if (rh)
    {
        *rh = (float)(dev->cal.H1_rH - dev->cal.H0_rH) * (raw_rh - dev->cal.H0_T0_OUT) /
                (float)(dev->cal.H1_T0_OUT - dev->cal.H0_T0_OUT) + (float)dev->cal.H0_rH;
        if (*rh > 100)
            *rh = 100;
    }
    if (t)
        *t = (float)(dev->cal.T1_degC - dev->cal.T0_degC) * (raw_t - dev->cal.T0_OUT) /
                (float)(dev->cal.T1_OUT - dev->cal.T0_OUT) + (float)dev->cal.T0_degC;

    return ESP_OK;
}

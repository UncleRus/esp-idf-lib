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

#include "hts221.h"

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <i2cdev.h>

#define I2C_FREQ_HZ 400000 // 400kHz

#define HTS221_REG_WHOAMI 0x0F         // R   Device identification register
#define HTS221_REG_AV_CONF 0x10        // R/W Humidity and temperature resolution mode
#define HTS221_REG_CTRL_REG1 0x20      // R/W Control register 1
#define HTS221_REG_CTRL_REG2 0x21      // R/W Control register 2
#define HTS221_REG_CTRL_REG3 0x22      // R/W Control register 3 for data ready output signal
#define HTS221_REG_STATUS_REG 0x27     // R   Status register
#define HTS221_REG_HUMIDITY_OUT_L 0x28 // R   Relative humidity output register (LSB)
#define HTS221_REG_HUMIDITY_OUT_H 0x29 // R   Relative humidity output register (MSB)
#define HTS221_REG_TEMP_OUT_L 0x2A     // R   Temperature output register (LSB)
#define HTS221_REG_TEMP_OUT_H 0x2B     // R   Temperature output register (MSB)
#define HTS221_REG_CALIB_START 0x30    // R/W Calibration start register

#define HTS221_AV_CONF_DEFAULT 0x1B // DEFAULT AV_CONF status register value according to datasheet

#define HTS221_CTRL_REG1 0x85 // Set CTRL_REG1 to power on mode, enable block data update, set output data rate to 1Hz
#define HTS221_CTRL_REG1_POWERON 0x80   // CTRL_REG1 status register value to power on HTS221
#define HTS221_CTRL_REG1_POWERDOWN 0x00 // CTRL_REG1 status register value to power on HTS221
#define HTS221_CTRL_REG2 0x00           // Default CTRL_REG2 status register value according to datasheet
#define HTS221_CTRL_REG3 0x00           // Default CTRL_REG3 status register value according to datasheet

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

static calibration_param_t calibration_parameters = {0};

esp_err_t hts221_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr != HTS221_I2C_ADDRESS)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t hts221_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t hts221_setup(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    uint8_t av_conf_default = HTS221_AV_CONF_DEFAULT;
    uint8_t ctrl_reg1 = HTS221_CTRL_REG1;
    uint8_t ctrl_reg2 = HTS221_CTRL_REG2;
    uint8_t ctrl_reg3 = HTS221_CTRL_REG3;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, HTS221_REG_AV_CONF, &av_conf_default, 1));
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, HTS221_REG_CTRL_REG1, &ctrl_reg1, 1));
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, HTS221_REG_CTRL_REG2, &ctrl_reg2, 1));
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, HTS221_REG_CTRL_REG3, &ctrl_reg3, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    ESP_ERROR_CHECK(hts221_read_calibration_coeff(dev));

    return ESP_OK;
}

esp_err_t hts221_read_calibration_coeff(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

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

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x30, &H0_rH_x2, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x31, &H1_rH_x2, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x32, &T0_degC_x8_lsb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x33, &T1_degC_x8_lsb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x35, &T0_T1_msb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x36, &H0_T0_OUT_lsb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x37, &H0_T0_OUT_msb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x3A, &H1_T0_OUT_lsb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x3B, &H1_T0_OUT_msb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x3C, &T0_OUT_lsb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x3D, &T0_OUT_msb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x3E, &T1_OUT_lsb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, 0x3F, &T1_OUT_msb, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    calibration_parameters.H0_rH = (H0_rH_x2 >> 1);
    calibration_parameters.H1_rH = (H1_rH_x2 >> 1);
    calibration_parameters.T0_degC = ((T0_T1_msb & 0x03) << 8 | T0_degC_x8_lsb) >> 3;
    calibration_parameters.T1_degC = ((T0_T1_msb & 0x0C) << 6 | T1_degC_x8_lsb) >> 3;
    calibration_parameters.H0_T0_OUT = (H0_T0_OUT_msb << 8) | H0_T0_OUT_lsb;
    calibration_parameters.H1_T0_OUT = (H1_T0_OUT_msb << 8) | H1_T0_OUT_lsb;
    calibration_parameters.T0_OUT = (T0_OUT_msb << 8) | T0_OUT_lsb;
    calibration_parameters.T1_OUT = (T1_OUT_msb << 8) | T1_OUT_lsb;

    return ESP_OK;
}

esp_err_t hts221_read_data(i2c_dev_t *dev, float *temperature, float *humidity)
{
    CHECK_ARG(dev && (temperature || humidity));

    uint8_t status_reg;
    uint8_t humidity_msb;
    uint8_t humidity_lsb;
    uint8_t temperature_msb;
    uint8_t temperature_lsb;
    int16_t raw_humidity;
    int16_t raw_temperature;

    ESP_ERROR_CHECK(hts221_read_status_register(dev, &status_reg));
    status_reg = status_reg & 0x03;

    if (status_reg != 0x03)
    {
        ESP_LOGE(TAG, "No new measurment data is available");
        return ESP_ERR_INVALID_STATE;
    }

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, HTS221_REG_HUMIDITY_OUT_L, &humidity_lsb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, HTS221_REG_HUMIDITY_OUT_H, &humidity_msb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, HTS221_REG_TEMP_OUT_L, &temperature_lsb, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, HTS221_REG_TEMP_OUT_H, &temperature_msb, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    raw_humidity = (humidity_msb << 8) | humidity_lsb;
    raw_temperature = (temperature_msb << 8) | temperature_lsb;

    *humidity = (float)(calibration_parameters.H1_rH - calibration_parameters.H0_rH) *
                    (raw_humidity - calibration_parameters.H0_T0_OUT) /
                    (float)(calibration_parameters.H1_T0_OUT - calibration_parameters.H0_T0_OUT) +
                (float)calibration_parameters.H0_rH;
    if (*humidity > 100)
    {
        *humidity = 100;
    }
    *temperature = (float)(calibration_parameters.T1_degC - calibration_parameters.T0_degC) *
                       (raw_temperature - calibration_parameters.T0_OUT) /
                       (float)(calibration_parameters.T1_OUT - calibration_parameters.T0_OUT) +
                   (float)calibration_parameters.T0_degC;

    return ESP_OK;
}

esp_err_t hts221_who_am_i(i2c_dev_t *dev, uint8_t *who_am_i)
{
    CHECK_ARG(dev && who_am_i);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, HTS221_REG_WHOAMI, who_am_i, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t hts221_read_av_conf(i2c_dev_t *dev, hts221_temperature_avg_t *t_avg, hts221_humidity_avg_t *rh_avg)
{
    CHECK_ARG(dev && t_avg && rh_avg);

    uint8_t av_conf;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, HTS221_REG_AV_CONF, &av_conf, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *t_avg = (hts221_temperature_avg_t)((av_conf & 0x38) >> 3);
    *rh_avg = (hts221_humidity_avg_t)((av_conf & 0x07));

    return ESP_OK;
}

esp_err_t hts221_set_av_conf(i2c_dev_t *dev, hts221_temperature_avg_t t_avg, hts221_humidity_avg_t rh_avg)
{
    CHECK_ARG(dev && t_avg && rh_avg);

    uint8_t av_conf;

    av_conf = (uint8_t)((t_avg) << 3) | (rh_avg);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, HTS221_REG_AV_CONF, &av_conf, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t hts221_power_toggle(i2c_dev_t *dev, hts221_power_t *power)
{
    CHECK_ARG(dev && power);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, HTS221_REG_CTRL_REG1, (uint8_t *)power, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t hts221_heater_toggle(i2c_dev_t *dev, hts221_heater_t *heater)
{
    CHECK_ARG(dev && heater);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, HTS221_REG_CTRL_REG2, (uint8_t *)heater, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t hts221_read_status_register(i2c_dev_t *dev, uint8_t *status_reg)
{
    CHECK_ARG(dev && status_reg);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, HTS221_REG_STATUS_REG, status_reg, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
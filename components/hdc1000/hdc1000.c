/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file hdc1000.c
 *
 * ESP-IDF driver for HDC1000 temperature and humidity sensor
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "hdc1000.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#define I2C_FREQ_HZ 400000 // 400kHz

static const char *TAG = "hdc1000";

#define REG_TEMPERATURE 0x00
#define REG_HUMIDITY    0x01
#define REG_CONFIG      0x02
#define REG_SERIAL_H    0xfb
#define REG_SERIAL_M    0xfc
#define REG_SERIAL_L    0xfd
#define REG_MANUF_ID    0xfe
#define REG_DEVICE_ID   0xff

#define BIT_CONFIG_RST  15
#define BIT_CONFIG_HEAT 13
#define BIT_CONFIG_MODE 12
#define BIT_CONFIG_BTST 11
#define BIT_CONFIG_TRES 10
#define BIT_CONFIG_HRES 8

#define MASK_CONFIG_HRES (3 << BIT_CONFIG_HRES)

#define RESET_TIMEOUT_US (200 * 1000)
#define MEASURE_TIMEOUT_MS 10 // 6.5 ms

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static inline uint16_t shuffle(uint16_t v)
{
    return (v >> 8) | (v << 8);
}

static esp_err_t read_reg_nolock(hdc1000_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, reg, val, 2));
    *val = shuffle(*val);
    return ESP_OK;
}

static esp_err_t write_reg_nolock(hdc1000_t *dev, uint8_t reg, uint16_t val)
{
    uint16_t v = shuffle(val);
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &v, 2);
}

static esp_err_t update_reg_nolock(hdc1000_t *dev, uint8_t reg, uint16_t val, uint16_t mask)
{
    uint16_t v;
    CHECK(read_reg_nolock(dev, reg, &v));
    v = (v & mask) | (val & mask);
    return write_reg_nolock(dev, reg, v);
}

static esp_err_t read_reg(hdc1000_t *dev, uint8_t reg, uint16_t *val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

static esp_err_t update_reg(hdc1000_t *dev, uint8_t reg, uint16_t val, uint16_t mask)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, update_reg_nolock(dev, reg, val, mask));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

static float calc_temperature(uint8_t *buf)
{
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return raw / 65536.0f * 165.0f - 40;
}

static float calc_humidity(uint8_t *buf)
{
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return raw / 65536.0f * 100.0f;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t hdc1000_init_desc(hdc1000_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr < HDC1000_I2C_ADDRESS_0 || addr > HDC1000_I2C_ADDRESS_3)
    {
        ESP_LOGE(TAG, "Invalid I2C address: 0x%02x", addr);
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

esp_err_t hdc1000_free_desc(hdc1000_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t hdc1000_init(hdc1000_t *dev)
{
    CHECK_ARG(dev);

    esp_err_t res = ESP_OK;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // reset
    I2C_DEV_CHECK(&dev->i2c_dev, update_reg_nolock(dev, REG_CONFIG, BIT(BIT_CONFIG_RST), BIT(BIT_CONFIG_RST)));
    // wait
    int64_t start = esp_timer_get_time();
    while (true)
    {
        if ((esp_timer_get_time() - start) >= RESET_TIMEOUT_US)
        {
            ESP_LOGE(TAG, "Timeout while reseting device");
            res = ESP_ERR_TIMEOUT;
            goto exit;
        }
        uint16_t r;
        I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, REG_CONFIG, &r));
        if (!(r & BIT(BIT_CONFIG_RST)))
            break;
        vTaskDelay(1);
    }
    dev->mode = HDC1000_MEASURE_BOTH;

exit:
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return res;
}

esp_err_t hdc1000_get_serial(hdc1000_t *dev, uint64_t *serial)
{
    CHECK_ARG(dev && serial);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    *serial = 0;

    uint16_t *dst = (uint16_t *)serial;
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, REG_SERIAL_L, dst));
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, REG_SERIAL_M, ++dst));
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, REG_SERIAL_H, ++dst));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t hdc1000_get_manufacturer_id(hdc1000_t *dev, uint16_t *id)
{
    CHECK_ARG(dev && id);

    return read_reg(dev, REG_MANUF_ID, id);
}

esp_err_t hdc1000_get_device_id(hdc1000_t *dev, uint16_t *id)
{
    CHECK_ARG(dev && id);

    return read_reg(dev, REG_DEVICE_ID, id);
}

esp_err_t hdc1000_get_battery_status(hdc1000_t *dev, bool *undervolt)
{
    CHECK_ARG(dev && undervolt);

    uint16_t v;
    CHECK(read_reg(dev, REG_CONFIG, &v));
    *undervolt = v & BIT(BIT_CONFIG_BTST) ? true : false;

    return ESP_OK;
}

esp_err_t hdc1000_get_heater(hdc1000_t *dev, bool *on)
{
    CHECK_ARG(dev && on);

    uint16_t v;
    CHECK(read_reg(dev, REG_CONFIG, &v));
    *on = v & BIT(BIT_CONFIG_HEAT) ? true : false;

    return ESP_OK;
}

esp_err_t hdc1000_set_heater(hdc1000_t *dev, bool on)
{
    CHECK_ARG(dev);

    return update_reg(dev, REG_CONFIG, on ? BIT(BIT_CONFIG_HEAT) : 0, BIT(BIT_CONFIG_HEAT));
}

esp_err_t hdc1000_set_measurement_mode(hdc1000_t *dev, hdc1000_measurement_mode_t mode)
{
    CHECK_ARG(dev && mode <= HDC1000_MEASURE_BOTH);

    dev->mode = mode;
    return update_reg(dev, REG_CONFIG, mode == HDC1000_MEASURE_BOTH ? BIT(BIT_CONFIG_MODE) : 0, BIT(BIT_CONFIG_MODE));
}

esp_err_t hdc1000_get_resolution(hdc1000_t *dev, hdc1000_temperature_resolution_t *tres, hdc1000_humidity_resolution_t *hres)
{
    CHECK_ARG(dev && (tres || hres));

    uint16_t v;
    CHECK(read_reg(dev, REG_CONFIG, &v));

    if (tres)
        *tres = (v >> BIT_CONFIG_TRES) & 1;
    if (hres)
        *hres = (v >> BIT_CONFIG_HRES) & 3;

    return ESP_OK;
}

esp_err_t hdc1000_set_resolution(hdc1000_t *dev, hdc1000_temperature_resolution_t tres, hdc1000_humidity_resolution_t hres)
{
    CHECK_ARG(dev && tres <= HDC1000_T_RES_11 && hres <= HDC1000_H_RES_8);

    return update_reg(dev, REG_CONFIG, (tres << BIT_CONFIG_TRES) | (hres << BIT_CONFIG_HRES),
            BIT(BIT_CONFIG_TRES) | MASK_CONFIG_HRES);
}

esp_err_t hdc1000_trigger_measurement(hdc1000_t *dev)
{
    CHECK_ARG(dev);

    uint8_t r = dev->mode == HDC1000_MEASURE_HUMIDITY ? REG_HUMIDITY : REG_TEMPERATURE;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, NULL, 0, &r, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t hdc1000_get_data(hdc1000_t *dev, float *t, float *rh)
{
    CHECK_ARG(dev && (t || rh));

    if ((!t && dev->mode == HDC1000_MEASURE_TEMPERATURE)
            || (!rh && dev->mode == HDC1000_MEASURE_HUMIDITY))
    {
        ESP_LOGE(TAG, "Measurement mode does not match hdc1000_get_data() arguments");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[4] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, buf, dev->mode == HDC1000_MEASURE_BOTH ? 4 : 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    switch (dev->mode)
    {
        case HDC1000_MEASURE_TEMPERATURE:
            *t = calc_temperature(buf);
            break;
        case HDC1000_MEASURE_HUMIDITY:
            *rh = calc_humidity(buf);
            break;
        case HDC1000_MEASURE_BOTH:
            if (t)
                *t = calc_temperature(buf);
            if (rh)
                *rh = calc_humidity(buf + 2);
            break;
    }

    return ESP_OK;
}

esp_err_t hdc1000_measure(hdc1000_t *dev, float *t, float *rh)
{
    CHECK(hdc1000_trigger_measurement(dev));
    vTaskDelay(pdMS_TO_TICKS(dev->mode == HDC1000_MEASURE_BOTH ? 2 * MEASURE_TIMEOUT_MS : MEASURE_TIMEOUT_MS));
    return hdc1000_get_data(dev, t, rh);
}

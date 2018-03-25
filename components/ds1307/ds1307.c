/**
 * @file ds1307.c
 *
 * Driver for DS1307 RTC
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2016 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#include "ds1307.h"
#include <esp_err.h>

#define I2C_FREQ_HZ 400000

#define RAM_SIZE 56

#define TIME_REG    0
#define CONTROL_REG 7
#define RAM_REG     8

#define CH_BIT      (1 << 7)
#define HOUR12_BIT  (1 << 6)
#define PM_BIT      (1 << 5)
#define SQWE_BIT    (1 << 4)
#define OUT_BIT     (1 << 7)

#define CH_MASK      0x7f
#define SECONDS_MASK 0x7f
#define HOUR12_MASK  0x1f
#define HOUR24_MASK  0x3f
#define SQWEF_MASK   0x03
#define SQWE_MASK    0xef
#define OUT_MASK     0x7f

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

static inline esp_err_t read_register(i2c_port_t bus, uint8_t reg, uint8_t *val)
{
    return i2c_read_register(bus, DS1307_ADDR, reg, val, 1);
}

static esp_err_t update_register(i2c_port_t bus, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t old;
    esp_err_t res = read_register(bus, reg, &old);
    if (res != ESP_OK)
        return res;
    uint8_t buf = (old & mask) | val;
    return i2c_write_register(bus, DS1307_ADDR, reg, &buf, 1);
}

esp_err_t ds1307_i2c_init(i2c_port_t i2c_num, gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    return i2c_setup_master(i2c_num, scl_pin, sda_pin, I2C_FREQ_HZ);
}

esp_err_t ds1307_start(i2c_port_t bus, bool start)
{
    return update_register(bus, TIME_REG, CH_MASK, start ? 0 : CH_BIT);
}

esp_err_t ds1307_is_running(i2c_port_t bus, bool *running)
{
    uint8_t val;
    esp_err_t res = read_register(bus, TIME_REG, &val);
    if (res == ESP_OK)
        *running = val & CH_BIT;
    return res;
}

esp_err_t ds1307_get_time(i2c_port_t bus, struct tm *time)
{
    uint8_t buf[7];
    uint8_t reg = TIME_REG;

    esp_err_t res = i2c_read_register(bus, DS1307_ADDR, reg, buf, 7);
    if (res != ESP_OK)
        return res;

    time->tm_sec = bcd2dec(buf[0] & SECONDS_MASK);
    time->tm_min = bcd2dec(buf[1]);
    if (buf[2] & HOUR12_BIT)
    {
        // RTC in 12-hour mode
        time->tm_hour = bcd2dec(buf[2] & HOUR12_MASK) - 1;
        if (buf[2] & PM_BIT)
            time->tm_hour += 12;
    }
    else
        time->tm_hour = bcd2dec(buf[2] & HOUR24_MASK);
    time->tm_wday = bcd2dec(buf[3]) - 1;
    time->tm_mday = bcd2dec(buf[4]);
    time->tm_mon  = bcd2dec(buf[5]) - 1;
    time->tm_year = bcd2dec(buf[6]) + 2000;

    return ESP_OK;
}

esp_err_t ds1307_set_time(i2c_port_t bus, const struct tm *time)
{
    uint8_t buf[8];
    buf[0] = TIME_REG;
    buf[1] = dec2bcd(time->tm_sec);
    buf[2] = dec2bcd(time->tm_min);
    buf[3] = dec2bcd(time->tm_hour);
    buf[4] = dec2bcd(time->tm_wday + 1);
    buf[5] = dec2bcd(time->tm_mday);
    buf[6] = dec2bcd(time->tm_mon + 1);
    buf[7] = dec2bcd(time->tm_year - 2000);

    return i2c_write_register(bus, DS1307_ADDR, buf[0], &buf[1], 7);
}

esp_err_t ds1307_enable_squarewave(i2c_port_t bus, bool enable)
{
    return update_register(bus, CONTROL_REG, SQWE_MASK, enable ? SQWE_BIT : 0);
}

esp_err_t ds1307_is_squarewave_enabled(i2c_port_t bus, bool *sqw_en)
{
    uint8_t val;
    esp_err_t res = read_register(bus, CONTROL_REG, &val);
    if (res == ESP_OK)
        *sqw_en = val & SQWE_BIT;
    return res;
}

esp_err_t ds1307_set_squarewave_freq(i2c_port_t bus, ds1307_squarewave_freq_t freq)
{
    return update_register(bus, CONTROL_REG, SQWEF_MASK, freq);
}

esp_err_t ds1307_get_squarewave_freq(i2c_port_t bus, ds1307_squarewave_freq_t *sqw_freq)
{
    uint8_t val;
    esp_err_t res = read_register(bus, CONTROL_REG, &val);
    if (res == ESP_OK)
        *sqw_freq = val & SQWEF_MASK;
    return res;
}

esp_err_t ds1307_get_output(i2c_port_t bus, bool *out)
{
    uint8_t val;
    esp_err_t res = read_register(bus, CONTROL_REG, &val);
    if (res == ESP_OK)
        *out = val & OUT_BIT;
    return res;
}

esp_err_t ds1307_set_output(i2c_port_t bus, bool value)
{
    return update_register(bus, CONTROL_REG, OUT_MASK, value ? OUT_BIT : 0);
}

esp_err_t ds1307_read_ram(i2c_port_t bus, uint8_t offset, uint8_t *buf, uint8_t len)
{
    if (offset + len > RAM_SIZE)
        return ESP_ERR_INVALID_SIZE;

    uint8_t reg = RAM_REG + offset;

    return i2c_read_register(bus, DS1307_ADDR, reg, buf, len);
}

esp_err_t ds1307_write_ram(i2c_port_t bus, uint8_t offset, uint8_t *buf, uint8_t len)
{
    if (offset + len > RAM_SIZE)
        return ESP_ERR_INVALID_SIZE;

    uint8_t reg = RAM_REG + offset;

    return i2c_write_register(bus, DS1307_ADDR, reg, buf, len);
}

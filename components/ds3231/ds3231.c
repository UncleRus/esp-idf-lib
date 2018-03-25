/**
 * @file ds3231.c
 *
 * ESP-IDF driver for DS3231 high precision RTC module
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2015 Richard A Burton <richardaburton@gmail.com>
 * Copyright (C) 2016 Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
 * MIT Licensed as described in the file LICENSE
 */

#include "ds3231.h"
#include <esp_err.h>

#define I2C_FREQ_HZ 400000

#define DS3231_STAT_OSCILLATOR 0x80
#define DS3231_STAT_32KHZ      0x08
#define DS3231_STAT_BUSY       0x04
#define DS3231_STAT_ALARM_2    0x02
#define DS3231_STAT_ALARM_1    0x01

#define DS3231_CTRL_OSCILLATOR    0x80
#define DS3231_CTRL_SQUAREWAVE_BB 0x40
#define DS3231_CTRL_TEMPCONV      0x20
#define DS3231_CTRL_SQWAVE_4096HZ 0x10
#define DS3231_CTRL_SQWAVE_1024HZ 0x08
#define DS3231_CTRL_SQWAVE_8192HZ 0x18
#define DS3231_CTRL_SQWAVE_1HZ    0x00
#define DS3231_CTRL_ALARM_INTS    0x04
#define DS3231_CTRL_ALARM2_INT    0x02
#define DS3231_CTRL_ALARM1_INT    0x01

#define DS3231_ALARM_WDAY   0x40
#define DS3231_ALARM_NOTSET 0x80

#define DS3231_ADDR_TIME    0x00
#define DS3231_ADDR_ALARM1  0x07
#define DS3231_ADDR_ALARM2  0x0b
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_ADDR_STATUS  0x0f
#define DS3231_ADDR_AGING   0x10
#define DS3231_ADDR_TEMP    0x11

#define DS3231_12HOUR_FLAG  0x40
#define DS3231_12HOUR_MASK  0x1f
#define DS3231_PM_FLAG      0x20
#define DS3231_MONTH_MASK   0x1f

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

/* Send a number of bytes to the rtc over i2c
 * returns true to indicate success
 */
static inline esp_err_t ds3231_send(i2c_port_t port, uint8_t reg, uint8_t *data, uint8_t len)
{
    return i2c_write_register(port, DS3231_ADDR, reg, data, len);
}

/* Read a number of bytes from the rtc over i2c
 * returns true to indicate success
 */
static inline esp_err_t ds3231_recv(i2c_port_t port, uint8_t reg, uint8_t *data, uint8_t len)
{
    return i2c_read_register(port, DS3231_ADDR, reg, data, len);
}

esp_err_t ds3231_i2c_init(i2c_port_t i2c_num, gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    return i2c_setup_master(i2c_num, scl_pin, sda_pin, I2C_FREQ_HZ);
}

esp_err_t ds3231_set_time(i2c_port_t port, struct tm *time)
{
    uint8_t data[7];

    /* time/date data */
    data[0] = dec2bcd(time->tm_sec);
    data[1] = dec2bcd(time->tm_min);
    data[2] = dec2bcd(time->tm_hour);
    /* The week data must be in the range 1 to 7, and to keep the start on the
     * same day as for tm_wday have it start at 1 on Sunday. */
    data[3] = dec2bcd(time->tm_wday + 1);
    data[4] = dec2bcd(time->tm_mday);
    data[5] = dec2bcd(time->tm_mon + 1);
    data[6] = dec2bcd(time->tm_year - 100);

    return ds3231_send(port, DS3231_ADDR_TIME, data, 7);
}

esp_err_t ds3231_set_alarm(i2c_port_t port, uint8_t alarms, struct tm *time1, uint8_t option1, struct tm *time2, uint8_t option2)
{
    int i = 0;
    uint8_t data[7];

    /* alarm 1 data */
    if (alarms != DS3231_ALARM_2)
    {
        data[i++] = (option1 >= DS3231_ALARM1_MATCH_SEC ? dec2bcd(time1->tm_sec) : DS3231_ALARM_NOTSET);
        data[i++] = (option1 >= DS3231_ALARM1_MATCH_SECMIN ? dec2bcd(time1->tm_min) : DS3231_ALARM_NOTSET);
        data[i++] = (option1 >= DS3231_ALARM1_MATCH_SECMINHOUR ? dec2bcd(time1->tm_hour) : DS3231_ALARM_NOTSET);
        data[i++] = (option1 == DS3231_ALARM1_MATCH_SECMINHOURDAY ? (dec2bcd(time1->tm_wday + 1) & DS3231_ALARM_WDAY) :
            (option1 == DS3231_ALARM1_MATCH_SECMINHOURDATE ? dec2bcd(time1->tm_mday) : DS3231_ALARM_NOTSET));
    }

    /* alarm 2 data */
    if (alarms != DS3231_ALARM_1)
    {
        data[i++] = (option2 >= DS3231_ALARM2_MATCH_MIN ? dec2bcd(time2->tm_min) : DS3231_ALARM_NOTSET);
        data[i++] = (option2 >= DS3231_ALARM2_MATCH_MINHOUR ? dec2bcd(time2->tm_hour) : DS3231_ALARM_NOTSET);
        data[i++] = (option2 == DS3231_ALARM2_MATCH_MINHOURDAY ? (dec2bcd(time2->tm_wday + 1) & DS3231_ALARM_WDAY) :
            (option2 == DS3231_ALARM2_MATCH_MINHOURDATE ? dec2bcd(time2->tm_mday) : DS3231_ALARM_NOTSET));
    }

    return ds3231_send(port, (alarms == DS3231_ALARM_2 ? DS3231_ADDR_ALARM2 : DS3231_ADDR_ALARM1), data, i);
}

/* Get a byte containing just the requested bits
 * pass the register address to read, a mask to apply to the register and
 * an uint* for the output
 * you can test this value directly as true/false for specific bit mask
 * of use a mask of 0xff to just return the whole register byte
 * returns true to indicate success
 */
static esp_err_t ds3231_get_flag(i2c_port_t port, uint8_t addr, uint8_t mask, uint8_t *flag)
{
    uint8_t data;

    /* get register */
    esp_err_t res = ds3231_recv(port, addr, &data, 1);
    if (res != ESP_OK)
        return res;

    /* return only requested flag */
    *flag = (data & mask);
    return ESP_OK;
}

/* Set/clear bits in a byte register, or replace the byte altogether
 * pass the register address to modify, a byte to replace the existing
 * value with or containing the bits to set/clear and one of
 * DS3231_SET/DS3231_CLEAR/DS3231_REPLACE
 * returns true to indicate success
 */
static esp_err_t ds3231_set_flag(i2c_port_t port, uint8_t addr, uint8_t bits, uint8_t mode)
{
    uint8_t data;

    /* get status register */
    esp_err_t res = ds3231_recv(port, addr, &data, 1);
    if (res != ESP_OK)
        return res;
    /* clear the flag */
    if (mode == DS3231_REPLACE)
        data = bits;
    else if (mode == DS3231_SET)
        data |= bits;
    else
        data &= ~bits;

    return ds3231_send(port, addr, &data, 1);
}

esp_err_t ds3231_get_oscillator_stop_flag(i2c_port_t port, bool *flag)
{
    uint8_t f;

    esp_err_t res = ds3231_get_flag(port, DS3231_ADDR_STATUS, DS3231_STAT_OSCILLATOR, &f);
    if (res == ESP_OK)
        *flag = (f ? true : false);

    return res;
}

esp_err_t ds3231_clear_oscillator_stop_flag(i2c_port_t port)
{
    return ds3231_set_flag(port, DS3231_ADDR_STATUS, DS3231_STAT_OSCILLATOR, DS3231_CLEAR);
}

esp_err_t ds3231_get_alarm_flags(i2c_port_t port, uint8_t *alarms)
{
    return ds3231_get_flag(port, DS3231_ADDR_STATUS, DS3231_ALARM_BOTH, alarms);
}

esp_err_t ds3231_clear_alarm_flags(i2c_port_t port, uint8_t alarms)
{
    return ds3231_set_flag(port, DS3231_ADDR_STATUS, alarms, DS3231_CLEAR);
}

esp_err_t ds3231_enable_alarm_ints(i2c_port_t port, uint8_t alarms)
{
    return ds3231_set_flag(port, DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS | alarms, DS3231_SET);
}

esp_err_t ds3231_disable_alarm_ints(i2c_port_t port, uint8_t alarms)
{
    /* Just disable specific alarm(s) requested
     * does not disable alarm interrupts generally (which would enable the squarewave)
     */
    return ds3231_set_flag(port, DS3231_ADDR_CONTROL, alarms, DS3231_CLEAR);
}

esp_err_t ds3231_enable_32khz(i2c_port_t port)
{
    return ds3231_set_flag(port, DS3231_ADDR_STATUS, DS3231_STAT_32KHZ, DS3231_SET);
}

esp_err_t ds3231_disable_32khz(i2c_port_t port)
{
    return ds3231_set_flag(port, DS3231_ADDR_STATUS, DS3231_STAT_32KHZ, DS3231_CLEAR);
}

esp_err_t ds3231_enable_squarewave(i2c_port_t port)
{
    return ds3231_set_flag(port, DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS, DS3231_CLEAR);
}

esp_err_t ds3231_disable_squarewave(i2c_port_t port)
{
    return ds3231_set_flag(port, DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS, DS3231_SET);
}

esp_err_t ds3231_set_squarewave_freq(i2c_port_t port, uint8_t freq)
{
    uint8_t flag = 0;

    esp_err_t res = ds3231_get_flag(port, DS3231_ADDR_CONTROL, 0xff, &flag);
    if (res != ESP_OK)
        return res;
    /* clear current rate */
    flag &= ~DS3231_CTRL_SQWAVE_8192HZ;
    /* set new rate */
    flag |= freq;

    return ds3231_set_flag(port, DS3231_ADDR_CONTROL, flag, DS3231_REPLACE);
}

esp_err_t ds3231_get_raw_temp(i2c_port_t port, int16_t *temp)
{
    uint8_t data[2];

    data[0] = DS3231_ADDR_TEMP;
    esp_err_t res = ds3231_recv(port, DS3231_ADDR_TEMP,data, 2);
    if (res == ESP_OK)
        *temp = (int16_t)(int8_t)data[0] << 2 | data[1] >> 6;

    return res;
}

esp_err_t ds3231_get_temp_integer(i2c_port_t port, int8_t *temp)
{
    int16_t t_int;

    esp_err_t res = ds3231_get_raw_temp(port, &t_int);
    if (res == ESP_OK)
        *temp = t_int >> 2;

    return res;
}

esp_err_t ds3231_get_temp_float(i2c_port_t port, float *temp)
{
    int16_t t_int;

    esp_err_t res = ds3231_get_raw_temp(port, &t_int);
    if (res == ESP_OK)
        *temp = t_int * 0.25;

    return res;
}

esp_err_t ds3231_get_time(i2c_port_t port, struct tm *time)
{
    uint8_t data[7];

    /* read time */
    esp_err_t res = ds3231_recv(port, DS3231_ADDR_TIME, data, 7);
    if (res != ESP_OK)
        return res;

    /* convert to unix time structure */
    time->tm_sec = bcd2dec(data[0]);
    time->tm_min = bcd2dec(data[1]);
    if (data[2] & DS3231_12HOUR_FLAG)
    {
        /* 12H */
        time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
        /* AM/PM? */
        if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
    }
    else time->tm_hour = bcd2dec(data[2]); /* 24H */
    time->tm_wday = bcd2dec(data[3]) - 1;
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
    time->tm_year = bcd2dec(data[6]) + 100;
    time->tm_isdst = 0;

    // apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
    //applyTZ(time);

    return ESP_OK;
}

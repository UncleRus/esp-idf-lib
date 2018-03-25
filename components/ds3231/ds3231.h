/**
 * @file ds3231.h
 *
 * ESP-IDF driver for DS3231 high precision RTC module
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2015 Richard A Burton <richardaburton@gmail.com>
 * Copyright (C) 2016 Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
 * MIT Licensed as described in the file LICENSE
 */

#ifndef __DS3231_H__
#define __DS3231_H__

#include <time.h>
#include <stdbool.h>
#include <i2c_utils.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define DS3231_ADDR 0x68

enum {
    DS3231_SET = 0,
    DS3231_CLEAR,
    DS3231_REPLACE
};

enum {
    DS3231_ALARM_NONE = 0,
    DS3231_ALARM_1,
    DS3231_ALARM_2,
    DS3231_ALARM_BOTH
};

enum {
    DS3231_ALARM1_EVERY_SECOND = 0,
    DS3231_ALARM1_MATCH_SEC,
    DS3231_ALARM1_MATCH_SECMIN,
    DS3231_ALARM1_MATCH_SECMINHOUR,
    DS3231_ALARM1_MATCH_SECMINHOURDAY,
    DS3231_ALARM1_MATCH_SECMINHOURDATE
};

enum {
    DS3231_ALARM2_EVERY_MIN = 0,
    DS3231_ALARM2_MATCH_MIN,
    DS3231_ALARM2_MATCH_MINHOUR,
    DS3231_ALARM2_MATCH_MINHOURDAY,
    DS3231_ALARM2_MATCH_MINHOURDATE
};

esp_err_t ds3231_i2c_init(i2c_port_t i2c_num, gpio_num_t scl_pin, gpio_num_t sda_pin);

/**
 * @brief Set the time on the rtc
 * timezone agnostic, pass whatever you like
 * I suggest using GMT and applying timezone and DST when read back
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_set_time(i2c_port_t port, struct tm *time);

/**
 * @brief Set alarms
 * alarm1 works with seconds, minutes, hours and day of week/month, or fires every second
 * alarm2 works with minutes, hours and day of week/month, or fires every minute
 * not all combinations are supported, see DS3231_ALARM1_* and DS3231_ALARM2_* defines
 * for valid options you only need to populate the fields you are using in the tm struct,
 * and you can set both alarms at the same time (pass DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH)
 * if only setting one alarm just pass 0 for tm struct and option field for the other alarm
 * if using DS3231_ALARM1_EVERY_SECOND/DS3231_ALARM2_EVERY_MIN you can pass 0 for tm stuct
 * if you want to enable interrupts for the alarms you need to do that separately
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_set_alarm(i2c_port_t port, uint8_t alarms, struct tm *time1, uint8_t option1, struct tm *time2, uint8_t option2);

/**
 * @brief Check if oscillator has previously stopped, e.g. no power/battery or disabled
 * sets flag to true if there has been a stop
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_oscillator_stop_flag(i2c_port_t port, bool *flag);

/**
 * @brief Clear the oscillator stopped flag
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_clear_oscillator_stop_flag(i2c_port_t port);

/**
 * @brief Check which alarm(s) have past
 * sets alarms to DS3231_ALARM_NONE/DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_alarm_flags(i2c_port_t port, uint8_t *alarms);

/**
 * @brief Clear alarm past flag(s)
 * pass DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_clear_alarm_flags(i2c_port_t port, uint8_t alarms);

/**
 * @brief enable alarm interrupts (and disables squarewave)
 * pass DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH
 * if you set only one alarm the status of the other is not changed
 * you must also clear any alarm past flag(s) for alarms with
 * interrupt enabled, else it will trigger immediately
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_enable_alarm_ints(i2c_port_t port, uint8_t alarms);

/**
 * @brief Disable alarm interrupts (does not (re-)enable squarewave)
 * pass DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_disable_alarm_ints(i2c_port_t port, uint8_t alarms);

/**
 * @brief Enable the output of 32khz signal
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_enable_32khz(i2c_port_t port);

/**
 * @brief Disable the output of 32khz signal
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_disable_32khz(i2c_port_t port);

/**
 * @brief Enable the squarewave output (disables alarm interrupt functionality)
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_enable_squarewave(i2c_port_t port);

/**
 * @brief Disable the squarewave output (which re-enables alarm interrupts, but individual
 * alarm interrupts also need to be enabled, if not already, before they will trigger)
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_disable_squarewave(i2c_port_t port);

/**
 * @brief Set the frequency of the squarewave output (but does not enable it)
 * pass DS3231_SQUAREWAVE_RATE_1HZ/DS3231_SQUAREWAVE_RATE_1024HZ/DS3231_SQUAREWAVE_RATE_4096HZ/DS3231_SQUAREWAVE_RATE_8192HZ
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_set_squarewave_freq(i2c_port_t port, uint8_t freq);

/**
 * @brief Get the raw value
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_raw_temp(i2c_port_t port, int16_t *temp);

/**
 * @brief Get the temperature as an integer
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_temp_integer(i2c_port_t port, int8_t *temp);

/**
 * @brief Get the temerapture as a float (in quarter degree increments)
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_temp_float(i2c_port_t port, float *temp);

/**
 * @brief Get the time from the rtc, populates a supplied tm struct
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_time(i2c_port_t port, struct tm *time);

#ifdef	__cplusplus
}
#endif

#endif  /* __DS3231_H__ */

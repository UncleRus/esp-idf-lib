/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file pcf8563.h
 * @defgroup pcf8563 pcf8563
 * @{
 *
 * ESP-IDF driver for PCF8563 (BM8563) real-time clock/calendar
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __PCF8563_H__
#define __PCF8563_H__

#include <i2cdev.h>
#include <stdbool.h>
#include <time.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCF8563_I2C_ADDR 0x51

/**
 * Frequency output at pin CLKOUT
 */
typedef enum {
    PCF8563_DISABLED = 0, //!< CLKOUT output is inhibited and set high-impedance
    PCF8563_32768HZ,      //!< 32768 Hz
    PCF8563_1024HZ,       //!< 1024 Hz
    PCF8563_32HZ,         //!< 32 Hz
    PCF8563_1HZ,          //!< 1 Hz
} pcf8563_clkout_freq_t;

/**
 * Timer clock
 */
typedef enum {
    PCF8563_TIMER_4096HZ = 0, //!< 4096 Hz
    PCF8563_TIMER_64HZ,       //!< 64 Hz
    PCF8563_TIMER_1HZ,        //!< 1 Hz
    PCF8563_TIMER_1_60HZ      //!< 1/60 Hz
} pcf8563_timer_clock_t;

/**
 * Flags to setup alarm
 */
typedef enum {
    PCF8563_ALARM_MATCH_MIN     = 0x01, //!< Alarm when minute matched
    PCF8563_ALARM_MATCH_HOUR    = 0x02, //!< Alarm when hour matched
    PCF8563_ALARM_MATCH_DAY     = 0x04, //!< Alarm when day matched
    PCF8563_ALARM_MATCH_WEEKDAY = 0x08  //!< Alarm when weekday matched
} pcf8563_alarm_flags_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev I2C device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_free_desc(i2c_dev_t *dev);

/**
 * @brief Set the time on the RTC
 *
 * @param dev I2C device descriptor
 * @param time Pointer to time struct
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_set_time(i2c_dev_t *dev, struct tm *time);

/**
 * @brief Get the time from the RTC
 *
 * @param dev I2C device descriptor
 * @param[out] time Pointer to time struct
 * @param[out] valid Time validity, false when RTC had power failures
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_get_time(i2c_dev_t *dev, struct tm *time, bool *valid);

/**
 * @brief Set output frequency on CLKOUT pin
 *
 * @param dev I2C device descriptor
 * @param freq Frequency
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_set_clkout(i2c_dev_t *dev, pcf8563_clkout_freq_t freq);

/**
 * @brief Get current frequency on CLKOUT pin
 *
 * @param dev I2C device descriptor
 * @param[out] freq Frequency
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_get_clkout(i2c_dev_t *dev, pcf8563_clkout_freq_t *freq);

/**
 * @brief Setup timer
 *
 * @param dev I2C device descriptor
 * @param int_enable true for enable interrupt on timer
 * @param clock Timer frequency
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_set_timer_settings(i2c_dev_t *dev, bool int_enable, pcf8563_timer_clock_t clock);

/**
 * @brief Get timer settings
 *
 * @param dev I2C device descriptor
 * @param[out] int_enabled true if timer interrupt is enabled
 * @param[out] clock Timer frequency
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_get_timer_settings(i2c_dev_t *dev, bool *int_enabled, pcf8563_timer_clock_t *clock);

/**
 * @brief Set timer register value
 *
 * @param dev I2C device descriptor
 * @param value Value to set int timer register
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_set_timer_value(i2c_dev_t *dev, uint8_t value);

/**
 * @brief Get timer register value
 *
 * @param dev I2C device descriptor
 * @param[out] value Timer value
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_get_timer_value(i2c_dev_t *dev, uint8_t *value);

/**
 * @brief Start timer
 *
 * @param dev I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_start_timer(i2c_dev_t *dev);

/**
 * @brief Stop timer
 *
 * @param dev I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_stop_timer(i2c_dev_t *dev);

/**
 * @brief Get state of the timer flag
 *
 * @param dev I2C device descriptor
 * @param[out] timer true when flag is set
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_get_timer_flag(i2c_dev_t *dev, bool *timer);

/**
 * @brief Clear timer flag
 *
 * @param dev I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_clear_timer_flag(i2c_dev_t *dev);

/**
 * @brief Setup alarm
 *
 * @param dev I2C device descriptor
 * @param int_enable true to enable alarm interrupt
 * @param flags Alarm types, combination of pcf8563_alarm_flags_t values
 * @param time Alarm time. Only tm_min, tm_hour, tm_mday and tm_wday are used
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_set_alarm(i2c_dev_t *dev, bool int_enable, uint32_t flags, struct tm *time);

/**
 * @brief Get alarm settings
 *
 * @param dev I2C device descriptor
 * @param[out] int_enabled true if alarm interrupt is enabled
 * @param[out] flags Selected alarm types, combination of pcf8563_alarm_flags_t values
 * @param[out] time Alarm time. Only tm_min, tm_hour, tm_mday and tm_wday are used
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_get_alarm(i2c_dev_t *dev, bool *int_enabled, uint32_t *flags, struct tm *time);

/**
 * @brief Get alarm flag
 *
 * @param dev I2C device descriptor
 * @param[out] alarm true if alarm occurred
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_get_alarm_flag(i2c_dev_t *dev, bool *alarm);

/**
 * @brief Clear alarm flag
 *
 * @param dev I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf8563_clear_alarm_flag(i2c_dev_t *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PCF8563_H__ */

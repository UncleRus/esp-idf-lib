/**
 * @file ds1302.h
 *
 * ESP-IDF driver for DS1302 RTC
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016, 2019 Ruslan V. Uss <unclerus@gmail.com>\n
 * Copyright (C) 2016 Pavel Merzlyakov <merzlyakovpavel@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __DS1302_H__
#define __DS1302_H__

#include <driver/gpio.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DS1302_RAM_SIZE 31

/**
 * Device descriptor
 */
typedef struct
{
    gpio_num_t ce_pin;     //!< GPIO pin connected to CE
    gpio_num_t io_pin;     //!< GPIO pin connected to chip I/O
    gpio_num_t sclk_pin;   //!< GPIO pin connected to SCLK
    bool ch;               //!< true if clock is halted
} ds1302_t;

esp_err_t ds1302_init(ds1302_t *dev);

/**
 * @brief Start/stop clock
 * @param start Start clock if true
 * @return False if RTC is write-protected
 */
esp_err_t ds1302_start(ds1302_t *dev, bool start);

/**
 * @brief Get current clock state
 * @return true if clock running
 */
esp_err_t ds1302_is_running(ds1302_t *dev, bool *running);

/**
 * @brief Enable/disable write protection
 * @param wp Set RTC write-protected if true
 */
esp_err_t ds1302_set_write_protect(ds1302_t *dev, bool wp);

/**
 * @brief Get write protection status
 * @return true if RTC write-protected
 */
esp_err_t ds1302_get_write_protect(ds1302_t *dev, bool *wp);

/**
 * @brief Get current time
 * @param time Pointer to the time struct to fill
 */
esp_err_t ds1302_get_time(ds1302_t *dev, struct tm *time);

/**
 * @brief Set time to RTC
 * @param time Pointer to the time struct
 * @return False if RTC is write-protected
 */
esp_err_t ds1302_set_time(ds1302_t *dev, const struct tm *time);

/**
 * @brief Read RAM contents into the buffer
 * @param offset Start byte, 0..55
 * @param buf Buffer
 * @param len Bytes to read, 1..56
 * @return false if error occured (invalid offset or buffer too big)
 */
esp_err_t ds1302_read_sram(ds1302_t *dev, uint8_t offset, void *buf, uint8_t len);

/**
 * @brief Write buffer to RTC RAM
 * @param offset Start byte, 0..55
 * @param buf Buffer
 * @param len Bytes to write, 1..56
 * @return false if error occured (invalid offset or buffer too big)
 */
esp_err_t ds1302_write_sram(ds1302_t *dev, uint8_t offset, void *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DS1302_H__ */

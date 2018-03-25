/**
 * @file ds1307.h
 *
 * Driver for DS1307 RTC
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#ifndef EXTRAS_DS1307_H_
#define EXTRAS_DS1307_H_

#include <stdbool.h>
#include <time.h>
#include <i2c_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DS1307_ADDR 0x68

/**
 * Squarewave frequency
 */
typedef enum
{
    DS1307_1HZ = 0, //!< 1 Hz
    DS1307_4096HZ,  //!< 4096 Hz
    DS1307_8192HZ,  //!< 8192 Hz
    DS1307_32768HZ  //!< 32768 Hz
} ds1307_squarewave_freq_t;

esp_err_t ds1307_i2c_init(i2c_port_t i2c_num, gpio_num_t scl_pin, gpio_num_t sda_pin);

/**
 * @brief Start/stop clock
 * @param start Start clock if true
 */
esp_err_t ds1307_start(i2c_port_t bus, bool start);

/**
 * @brief Get current clock state
 * @return true if clock running
 */
esp_err_t ds1307_is_running(i2c_port_t bus, bool *running);

/**
 * @brief Get current time
 * @param time Pointer to the time struct to fill
 */
esp_err_t ds1307_get_time(i2c_port_t bus, struct tm *time);

/**
 * @brief Set time to RTC
 * @param time Pointer to the time struct
 */
esp_err_t ds1307_set_time(i2c_port_t bus, const struct tm *time);

/**
 * @brief Enable or disable square-wave oscillator output
 * @param enable Enable oscillator if true
 */
esp_err_t ds1307_enable_squarewave(i2c_port_t bus, bool enable);

/**
 * @brief Get square-wave oscillator output
 * @return true if square-wave oscillator enabled
 */
esp_err_t ds1307_is_squarewave_enabled(i2c_port_t bus, bool *sqw_en);

/**
 * @brief Set square-wave oscillator frequency
 * @param freq Frequency
 */
esp_err_t ds1307_set_squarewave_freq(i2c_port_t bus, ds1307_squarewave_freq_t freq);

/**
 * @brief Get current square-wave oscillator frequency
 * @return Frequency
 */
esp_err_t ds1307_get_squarewave_freq(i2c_port_t bus, ds1307_squarewave_freq_t *sqw_freq);

/**
 * @brief Get current output level of the SQW/OUT pin
 * @param bus I2C port number
 * @param out current output level of the SQW/OUT pin, true if high
 * @return ESP_OK if no error occured
 */
esp_err_t ds1307_get_output(i2c_port_t bus, bool *out);

/**
 * @brief Set output level of the SQW/OUT pin
 * Set output level if square-wave output is disabled
 * @param value High level if true
 * @return ESP_OK if no error occured
 */
esp_err_t ds1307_set_output(i2c_port_t bus, bool value);

/**
 * @brief Read RAM contents into the buffer
 * @param offset Start byte, 0..55
 * @param buf Buffer
 * @param len Bytes to read, 1..56
 * @return ESP_OK if no error occured
 */
esp_err_t ds1307_read_ram(i2c_port_t bus, uint8_t offset, uint8_t *buf, uint8_t len);

/**
 * @brief Write buffer to RTC RAM
 * @param offset Start byte, 0..55
 * @param buf Buffer
 * @param len Bytes to write, 1..56
 * @return ESP_OK if no error occured
 */
esp_err_t ds1307_write_ram(i2c_port_t bus, uint8_t offset, uint8_t *buf, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* EXTRAS_DS1307_H_ */

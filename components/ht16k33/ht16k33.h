/*
 * Copyright (c) 2022 Timofei Korostelev <timofei_public@dranik.dev>
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
 * @file ht16k33.h
 * @defgroup ht16k33 ht16k33
 * @{
 *
 * Holtek HT16K33 LED Controller driver.
 * No keyscan features implemented.
 * Tip: PWM brightness frequency depends on I2C clock.
 *
 * Manufacturer link: https://www.holtek.com/productdetail/-/vg/HT16K33
 * Datasheet: https://www.holtek.com/documents/10179/116711/HT16K33v120.pdf
 *
 */

#if !defined(__HT16K33_H__)
#define __HT16K33_H__

#ifdef __cplusplus

extern "C" {
#endif

#define HT16K33_DEFAULT_ADDR 0x70
#define HT16K33_DEFAULT_I2C_FREQ_HZ 400000 // 400kHz
#define HT16K33_MAX_BRIGHTNESS 15
#define HT16K33_RAM_SIZE_BYTES 16

// Display blinking frequencies.
#define HT16K33_BLINKING_0HZ 0b00
#define HT16K33_BLINKING_2HZ 0b01
#define HT16K33_BLINKING_1HZ 0b10
#define HT16K33_BLINKING_05HZ 0b11

#include <esp_err.h>
#include <driver/gpio.h>

#include <i2cdev.h>

/**
 * @brief Initialize HT16K33 device.
 *
 * @param dev output I2C device descriptor
 * @param port I2C port
 * @param i2c_freq I2C bus frequency
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @param addr device I2C address
 * @return ESP_OK in case of success
 */
esp_err_t ht16k33_init(i2c_dev_t *dev, i2c_port_t port, uint32_t i2c_freq_hz,
 gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t addr);

/**
 * @brief Free device descriptor.
 *
 * @param dev I2C device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ht16k33_free_desc(i2c_dev_t *dev);

/**
 * @brief Set brightness. Thread safe.
 *
 * @param dev I2C device descriptor
 * @param brightness Brighness value in 0-15 range.
 * @return ESP_OK to indicate success
 */
esp_err_t ht16k33_set_brightness(i2c_dev_t *dev, uint8_t brightness);

/**
 * @brief Display setup. ON/OFF and blinkng frequency. Thread safe.
 *
 * @param dev I2C device descriptor
 * @param on_flag On flag, 0 or 1.
 * @param blinking Blinking frequence. Limited options. See HT16K33_BLINKING_* constants.
 * @return ESP_OK to indicate success
 */
esp_err_t ht16k33_display_setup(i2c_dev_t *dev, uint8_t on_flag, uint8_t blinking);

/**
 * @brief Write whole HT16K33_RAM_SIZE_BYTES into RAM. Thread safe.
 *
 * @param data Bytes to write.
 * @return ESP_OK to indicate success
 */
esp_err_t ht16k33_ram_write(i2c_dev_t *dev, uint8_t *data);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __HT16K33__H__

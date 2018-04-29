/**
 * @file max7219.h
 *
 * ESP-IDF driver for MAX7219/MAX7221
 * Serially Interfaced, 8-Digit LED Display Drivers
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2017, 2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MAX7219_H__
#define __MAX7219_H__

#include <stdint.h>
#include <stdbool.h>
#include <driver/spi_master.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX7219_MAX_CASCADE_SIZE 8
#define MAX7219_MAX_BRIGHTNESS   31

/**
 * Display descriptor
 */
typedef struct
{
    spi_device_interface_config_t spi_cfg;
    spi_device_handle_t spi_dev;
    uint8_t digits;              //!< Accessible digits in 7seg. Up to cascade_size * 8
    uint8_t cascade_size;        //!< Up to `MAX7219_MAX_CASCADE_SIZE` MAX721xx cascaded
    bool mirrored;               //!< true for horizontally mirrored displays
    bool bcd;
} max7219_t;

/**
 * Initialize device descriptor
 * @param dev Device descriptor
 * @param host SPI host
 * @param cs_pin CS GPIO number
 * @return `ESP_OK` on success
 */
esp_err_t max7219_init_desc(max7219_t *dev, spi_host_device_t host, gpio_num_t cs_pin);

/**
 * Free device descriptor
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max7219_free_desc(max7219_t *dev);

/**
 * @brief Initialize display
 * Switch it to normal operation from shutdown mode,
 * set scan limit to the max and clear
 * @param dev Display descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max7219_init(max7219_t *dev);

/**
 * Set decode mode and clear display
 * @param dev Display descriptor
 * @param bcd true to set BCD decode mode, false to normal
 * @return `ESP_OK` on success
 */
esp_err_t max7219_set_decode_mode(max7219_t *dev, bool bcd);

/**
 * Set display brightness
 * @param dev Display descriptor
 * @param value Brightness value, 0..MAX7219_MAX_BRIGHTNESS
 * @return `ESP_OK` on success
 */
esp_err_t max7219_set_brightness(max7219_t *dev, uint8_t value);

/**
 * Shutdown display or set it to normal mode
 * @param dev Display descriptor
 * @param shutdown Shutdown display if true
 * @return `ESP_OK` on success
 */
esp_err_t max7219_set_shutdown_mode(max7219_t *dev, bool shutdown);

/**
 * Write data to display digit
 * @param dev Display descriptor
 * @param digit Digit index, 0..dev->digits - 1
 * @param val Data
 * @return `ESP_OK` on success
 */
esp_err_t max7219_set_digit(max7219_t *dev, uint8_t digit, uint8_t val);

/**
 * Clear display
 * @param dev Display descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max7219_clear(max7219_t *dev);

/**
 * Draw text on 7-segment display
 * @param dev Display descriptor
 * @param pos Start digit
 * @param s Text
 * @return `ESP_OK` on success
 */
esp_err_t max7219_draw_text_7seg(max7219_t *dev, uint8_t pos, const char *s);

/**
 * Draw 64-bit image on 8x8 matrix
 * @param dev Display descriptor
 * @param pos Start digit
 * @param image 64-bit buffer with image data
 * @return `ESP_OK` on success
 */
esp_err_t max7219_draw_image_8x8(max7219_t *dev, uint8_t pos, const void *image);

#ifdef __cplusplus
}
#endif

#endif /* __MAX7219_H__ */

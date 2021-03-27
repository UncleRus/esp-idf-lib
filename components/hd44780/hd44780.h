/**
 * @file hd44780.h
 * @defgroup hd44780 hd44780
 * @{
 *
 * ESP-IDF driver for HD44780 compatible LCD text displays
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __HD44780_H__
#define __HD44780_H__

#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HD44780_NOT_USED 0xff

/**
 * LCD font type. Please refer to the datasheet
 * of your module.
 */
typedef enum
{
    HD44780_FONT_5X8 = 0,
    HD44780_FONT_5X10
} hd44780_font_t;

typedef struct hd44780 hd44780_t;

typedef esp_err_t (*hd44780_write_cb_t)(const hd44780_t *lcd, uint8_t data);

/**
 * LCD descriptor. Fill it before use.
 */
struct hd44780
{
    hd44780_write_cb_t write_cb; //!< Data write callback. Set it to NULL in case of direct LCD connection to GPIO
    struct
    {
        uint8_t rs;        //!< GPIO/register bit used for RS pin
        uint8_t e;         //!< GPIO/register bit used for E pin
        uint8_t d4;        //!< GPIO/register bit used for D4 pin
        uint8_t d5;        //!< GPIO/register bit used for D5 pin
        uint8_t d6;        //!< GPIO/register bit used for D5 pin
        uint8_t d7;        //!< GPIO/register bit used for D5 pin
        uint8_t bl;        //!< GPIO/register bit used for backlight. Set it `HD44780_NOT_USED` if no backlight used
    } pins;
    hd44780_font_t font;   //!< LCD Font type
    uint8_t lines;         //!< Number of lines for LCD. Many 16x1 LCD has two lines (like 8x2)
    bool backlight;        //!< Current backlight state
};

/**
 * @brief Init LCD
 *
 * Set cursor position to (0, 0)
 *
 * @param lcd LCD descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_init(const hd44780_t *lcd);

/**
 * @brief Control LCD
 *
 * On/off LCD, show/hide cursor, set cursor blink
 *
 * @param lcd LCD descriptor
 * @param on Switch LCD on if true
 * @param cursor Show cursor if true
 * @param cursor_blink Enable cursor blinking if true
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_control(const hd44780_t *lcd, bool on, bool cursor, bool cursor_blink);

/**
 * @brief Clear LCD
 *
 * Clear memory and move cursor to (0, 0)
 *
 * @param lcd LCD descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_clear(const hd44780_t *lcd);

/**
 * @brief Move cursor
 *
 * @param lcd LCD descriptor
 * @param col Column
 * @param line Line
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_gotoxy(const hd44780_t *lcd, uint8_t col, uint8_t line);

/**
 * @brief Write character at cursor position
 *
 * @param lcd LCD descriptor
 * @param c Character to write
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_putc(const hd44780_t *lcd, char c);

/**
 * @brief Write NULL-terminated string at cursor position
 *
 * @param lcd LCD descriptor
 * @param s String to write
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_puts(const hd44780_t *lcd, const char *s);

/**
 * @brief Switch backlight
 *
 * @param lcd LCD descriptor
 * @param on Turn backlight on if true
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_switch_backlight(hd44780_t *lcd, bool on);

/**
 * @brief Upload character data to the CGRAM
 *
 * After upload cursor will be moved to (0, 0).
 *
 * @param lcd LCD descriptor
 * @param num Character number (0..7)
 * @param data Character data: 8 or 10 bytes depending on the font
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_upload_character(const hd44780_t *lcd, uint8_t num, const uint8_t *data);

/**
 * @brief Scroll the display content to left by one character
 *
 * @param lcd LCD descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_scroll_left(const hd44780_t *lcd);

/**
 * @brief Scroll the display content to right by one character
 *
 * @param lcd LCD descriptor
 * @return `ESP_OK` on success
 */
esp_err_t hd44780_scroll_right(const hd44780_t *lcd);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __HD44780_H__ */

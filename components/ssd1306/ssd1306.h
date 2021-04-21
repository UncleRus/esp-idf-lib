/**
 * @file ssd1306.h
 * @defgroup ssd1306 ssd1306
 * @{
 *
 * ESP-IDF driver for SSD1306/SH1106 OLED displays.
 *
 * Ported from esp-open-rtos.
 *
 * Copyright (c) 2016, 2021 urx (https://github.com/urx),
 *                          Ruslan V. Uss (https://github.com/UncleRus)
 *                          Zaltora (https://github.com/Zaltora)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef _SSD1306__H_
#define _SSD1306__H_

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>

#if HELPER_TARGET_IS_ESP8266 && defined(CONFIG_SSD1306_PROTOCOL_SPI4)
#error ssd1306 driver on ESP8266 only supports I2C protocol
#endif

#ifdef CONFIG_SSD1306_PROTOCOL_I2C
#include <i2cdev.h>
#endif
#ifdef CONFIG_SSD1306_PROTOCOL_SPI4
#include <driver/gpio.h>
#include <driver/spi_master.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_SSD1306_PROTOCOL_I2C
/**
 * I2C addresses
 */
#define SSD1306_I2C_ADDR0 0x3c
#define SSD1306_I2C_ADDR1 0x3d
#endif

/**
 * SH1106 pump voltage value
 */
typedef enum
{
    SH1106_VOLTAGE_74 = 0, //!< 7.4 Volt
    SH1106_VOLTAGE_80,     //!< 8.0 Volt
    SH1106_VOLTAGE_84,     //!< 8.4 Volt
    SH1106_VOLTAGE_90      //!< 9.0 Volt
} sh1106_voltage_t;

/**
 * Screen type
 */
typedef enum
{
    SSD1306_CHIP = 0,
    SH1106_CHIP
} ssd1306_chip_t;

/**
 * Device descriptor
 */
typedef struct
{
    ssd1306_chip_t chip;
#ifdef CONFIG_SSD1306_PROTOCOL_I2C
    i2c_dev_t i2c_dev;
#endif
#ifdef CONFIG_SSD1306_PROTOCOL_SPI4
    spi_device_interface_config_t spi_cfg;
    spi_device_handle_t spi_dev;
    gpio_num_t dc_gpio;
#endif
    uint8_t width;               //!< Screen width, currently supported 128px, 96px
    uint8_t height;              //!< Screen height, currently supported 16px, 32px, 64px
    uint8_t *fb;                 //!< Framebuffer
} ssd1306_t;

/**
 * Addressing mode, see datasheet
 */
typedef enum
{
    SSD1306_ADDR_MODE_HORIZONTAL = 0,
    SSD1306_ADDR_MODE_VERTICAL,
    SSD1306_ADDR_MODE_PAGE
} ssd1306_mem_addr_mode_t;

/**
 * Drawing color
 */
typedef enum
{
    OLED_COLOR_TRANSPARENT = -1, //!< Transparent (not drawing)
    OLED_COLOR_BLACK = 0,        //!< Black (pixel off)
    OLED_COLOR_WHITE = 1,        //!< White (or blue, yellow, pixel on)
    OLED_COLOR_INVERT = 2,       //!< Invert pixel (XOR)
} ssd1306_color_t;

typedef enum {
    SSD1306_MODE_NORMAL = 0,
    SSD1306_MODE_FILL,
} ssd1306_display_mode_t;

#ifdef CONFIG_SSD1306_PROTOCOL_I2C
/**
 * @brief Initialize I2C device descriptor
 *
 * Default SCL frequency is 400kHz
 *
 * @param dev Pointer device descriptor
 * @param port I2C port number
 * @param addr I2C address
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return ESP_OK on success
 */
esp_err_t ssd1306_init_desc(ssd1306_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
#endif

#ifdef CONFIG_SSD1306_PROTOCOL_SPI4
/**
 * @brief Initialize SPI 4-wires device descriptor
 *
 * SPI clock frequency is 8MHz
 *
 * @param dev Pointer device descriptor
 * @param host SPI host (SPI2_HOST/SPI3_HOST)
 * @param cs_gpio CS GPIO
 * @param dc_gpio DC GPIO
 * @return ESP_OK on success
 */
esp_err_t ssd1306_init_desc(ssd1306_t *dev, spi_host_device_t host, gpio_num_t cs_gpio, gpio_num_t dc_gpio);
#endif

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t ssd1306_free_desc(ssd1306_t *dev);

/**
 * @brief Initialize device
 *
 * This function also allocates framebuffer memory
 *
 * @param dev Pointer to device descriptor
 * @return ESP_OK on success
 */
esp_err_t ssd1306_init(ssd1306_t *dev);

/**
 * @brief Free allocated memory
 *
 * @param dev Pointer to device descriptor
 * @return ESP_OK on success
 */
esp_err_t ssd1306_done(ssd1306_t *dev);

/**
 * @brief Turn display on or off.
 *
 * @param dev Pointer to device descriptor
 * @param on Turn on if true
 * @return ESP_OK on success
 */
esp_err_t ssd1306_display_on(ssd1306_t *dev, bool on);

/**
 * @brief Set Display Start Line register
 *
 * Set the Display Start Line register to determine starting address of
 * display RAM, by selecting a value from 0 to 63. With value equal to 0,
 * RAM row 0 is mapped to COM0. With value equal to 1, RAM row 1 is mapped
 * to COM0 and so on.
 * @param dev Pointer to device descriptor
 * @param start Start line, 0..63
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_display_start_line(ssd1306_t *dev, uint8_t start);

/**
 * @brief Set display offset
 *
 * @param dev Pointer to device descriptor
 * @param offset Offset, 0..63
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_display_offset(ssd1306_t *dev, uint8_t offset);

/**
 * @brief Select charge pump voltage
 *
 * @param dev Pointer to device descriptor
 * @param select Select charge pump voltage value
 * @return ESP_OK on success
 */
esp_err_t sh1106_set_charge_pump_voltage(ssd1306_t *dev, sh1106_voltage_t select);

/**
 * @brief Enable or disable the charge pump
 *
 * See application note in datasheet.
 *
 * @param dev Pointer to device descriptor
 * @param enabled Enable charge pump if true
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_charge_pump_enabled(ssd1306_t *dev, bool enabled);

/**
 * Set memory addressing mode
 *
 * @param dev Pointer to device descriptor
 * @param mode Addressing mode
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_mem_addr_mode(ssd1306_t *dev, ssd1306_mem_addr_mode_t mode);

/**
 * @brief Change mapping
 *
 * Change the mapping between the display data column address and the
 * segment driver.
 *
 * @param dev Pointer to device descriptor
 * @param on Enable segment remapping if true
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_segment_remapping_enabled(ssd1306_t *dev, bool on);

/**
 * @brief Set scan direction
 *
 * Set the scan direction of the COM output, allowing layout flexibility
 * in the OLED module design. Additionally, the display will show once
 * this command is issued. For example, if this command is sent during
 * normal display then the graphic display will be vertically flipped
 * immediately.
 *
 * @param dev Pointer to device descriptor
 * @param fwd Forward direction if true, backward otherwise
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_scan_direction_fwd(ssd1306_t *dev, bool fwd);

/**
 * @brief Set the COM signals pin configuration
 *
 * Set the COM signals pin configuration to match the OLED panel
 * hardware layout.
 *
 * @param dev Pointer to device descriptor
 * @param config Sequential COM pin configuration
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_com_pin_hw_config(ssd1306_t *dev, uint8_t config);

/**
 * @brief Set display contrast
 *
 * @param dev Pointer to device descriptor
 * @param contrast Contrast increases as the value increases.
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_contrast(ssd1306_t *dev, uint8_t contrast);

/**
 * @brief Set display inversion
 *
 * Set the display to be either normal or inverse. In normal display
 * a RAM data of 1 indicates an “ON” pixel while in inverse display a
 * RAM data of 0 indicates an “ON” pixel.
 *
 * @param dev Pointer to device descriptor
 * @param on Inverse display if true
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_inversion(ssd1306_t *dev, bool on);

/**
 * @brief Set the divide ratio of display clock and oscillator frequency
 *
 * @param dev Pointer to device descriptor
 * @param osc_freq Lower nibble - DCLK divide ratio,
 *                 High nibble - oscillator frequency
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_osc_freq(ssd1306_t *dev, uint8_t osc_freq);

/**
 * @brief Set multiplex ratio
 *
 * Switch the default 63 multiplex mode to any multiplex ratio,
 * ranging from 16 to 63. The output pads COM0~COM63 will be switched
 * to the corresponding COM signal.
 *
 * @param dev Pointer to device descriptor
 * @param ratio Multiplex ratio, 16..63
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_mux_ratio(ssd1306_t *dev, uint8_t ratio);

/**
 * @brief Specify column start and end addresses
 *
 * Specify column start address and end address of the display data RAM.
 * This command also sets the column address pointer to column start
 * address. This pointer is used to define the current read/write column
 * address in graphic display data RAM. If horizontal address increment mode
 * is enabled by ssd1306_set_mem_addr_mode(), after finishing read/write
 * one column data, it is incremented automatically to the next column
 * address. Whenever the column address pointer finishes accessing the
 * end column address, it is reset back to start column address and the
 * row address is incremented to the next row.
 *
 * @param dev Pointer to device descriptor
 * @param start Start RAM address
 * @param stop End RAM address
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_column_addr(ssd1306_t *dev, uint8_t start, uint8_t stop);

/**
 * @brief Specify page start and end addresses
 *
 * Specify page start address and end address of the display data RAM.
 * This command also sets the page address pointer to page start address.
 * This pointer is used to define the current read/write page address in
 * graphic display data RAM. If vertical address increment mode is enabled by
 * ssd1306_set_mem_addr_mode(), after finishing read/write one page data,
 * it is incremented automatically to the next page address. Whenever the page
 * address pointer finishes accessing the end page address, it is reset back
 * to start page address.
 *
 * @param dev Pointer to device descriptor
 * @param start Start RAM address
 * @param stop End RAM address
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_page_addr(ssd1306_t *dev, uint8_t start, uint8_t stop);

/**
 * @brief Set the duration of the pre-charge period
 *
 * The interval is counted in number of DCLK, where RESET equals 2 DCLKs.
 *
 * @param dev Pointer to device descriptor
 * @param prchrg Pre-charge period
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_precharge_period(ssd1306_t *dev, uint8_t prchrg);

/**
 * @brief Adjust the VCOMH regulator output
 *
 * @param dev Pointer to device descriptor
 * @param lvl Deselect level
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_deseltct_lvl(ssd1306_t *dev, uint8_t lvl);

/**
 * @brief Set display mode
 *
 * @param dev Pointer to device descriptor
 * @param light Force the entire display to be “ON if true
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_display_mode(ssd1306_t *dev, ssd1306_display_mode_t mode);

/**
 * @brief Draw one pixel
 *
 * This function does not actually change screen contents.
 * Call ::ssd1306_flush() to display framebuffer.
 *
 * @param dev Pointer to device descriptor
 * @param fb Pointer to framebuffer. Framebuffer size = width * height / 8
 * @param x X coordinate
 * @param y Y coordinate
 * @param color Color of the pixel
 * @return ESP_OK on success
 */
esp_err_t ssd1306_set_pixel(ssd1306_t *dev, uint8_t x, uint8_t y, ssd1306_color_t color);

/**
 * @brief Draw horizontal line
 *
 * This function does not actually change screen contents.
 * Call ::ssd1306_flush() to display framebuffer.
 *
 * @param dev Pointer to device descriptor
 * @param fb Pointer to framebuffer. Framebuffer size = width * height / 8
 * @param x X coordinate or starting (left) point
 * @param y Y coordinate or starting (left) point
 * @param w Line width
 * @param color Color of the line
 * @return ESP_OK on success
 */
esp_err_t ssd1306_draw_hline(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w, ssd1306_color_t color);

/**
 * @brief Draw vertical line
 *
 * This function does not actually change screen contents.
 * Call ::ssd1306_flush() to display framebuffer.
 *
 * @param dev Pointer to device descriptor
 * @param fb Pointer to framebuffer. Framebuffer size = width * height / 8
 * @param x X coordinate or starting (top) point
 * @param y Y coordinate or starting (top) point
 * @param h Line height
 * @param color Color of the line
 * @return ESP_OK on success
 */
esp_err_t ssd1306_draw_vline(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t h, ssd1306_color_t color);

/**
 * @brief Send local framebuffer to display
 *
 * @param dev Pointer to device descriptor
 * @return ESP_OK on success
 */
esp_err_t ssd1306_flush(ssd1306_t *dev);

/**
 * @brief Clear screen
 *
 * This function does not actually change screen contents.
 * Call ::ssd1306_flush() to display framebuffer.
 *
 * @param dev Pointer to device descriptor
 * @return ESP_OK on success
 */
esp_err_t ssd1306_clear(ssd1306_t *dev);

/**
 * @brief Load image in XBM-format to the framebuffer.
 *
 * This function does not actually change screen contents.
 * Call ::ssd1306_flush() to display framebuffer.
 *
 * @param dev Pointer to device descriptor
 * @param xbm XBM-image
 * @return ESP_OK on success
 */
esp_err_t ssd1306_load_xbm(ssd1306_t *dev, uint8_t *xbm);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // _SSD1306__H_

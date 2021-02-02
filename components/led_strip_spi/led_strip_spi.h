/**
 * @file led_strip.h
 * @defgroup led_strip_spi led_strip_spi
 * @{
 *
 * SPI-based ESP-IDF driver for SK9822 LED strips
 *
 * Copyright (C) 2020 Ruslan V. Uss <https://github.com/UncleRus>
 *               2021 Tomoyuki Sakurai <y@trombik.org>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __LED_STRIP_SPI_H__
#define __LED_STRIP_SPI_H__

#include <driver/gpio.h>
#include <esp_err.h>
#include <led_effect.h>
#include <esp_idf_lib_helpers.h>

#if HELPER_TARGET_IS_ESP32
#include <driver/spi_master.h>
#elif HELPER_TARGET_IS_ESP8266
#include <driver/spi.h>
#else
#error "Unsupported target"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * LED type
 */
typedef enum
{
    LED_STRIP_SPI_SK9822 = 0,
    LED_STRIP_SPI_MAX
} led_strip_spi_type_t;

/**
 * LED strip descriptor
 */

#if HELPER_TARGET_IS_ESP32
typedef struct
{
    size_t length;
    void *buf;
    spi_bus_config_t bus_config;
    spi_device_interface_config_t device_interface_config;
    spi_host_device_t host_device; // SPI2_HOST or SPI3_HOST
    spi_device_handle_t device_handle;
    spi_transaction_t transaction;
    int dma_chan; // 1 or 2
} led_strip_spi_t;
#endif

#if HELPER_TARGET_IS_ESP8266
typedef struct
{
    size_t length;
    void *buf;
    spi_interface_t bus_config;
    spi_host_t host_device; // only HSPI can be used here
    spi_trans_t transaction;
    spi_clk_div_t clk_div;

} led_strip_spi_t;
#endif
/*
 * IO_MUX pins for SPI buses
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html
 *
 * | Pin  | SPI2 | SPI3 |
 * |------|------|------|
 * | SCLK |  14  |  18  |
 * | MOSI |  13  |  23  |
 */
#define LED_STRIP_SPI2_SCLK_IO_NUM (14)
#define LED_STRIP_SPI2_MOSI_IO_NUM (13)
#define LED_STRIP_SPI3_SCLK_IO_NUM (18)
#define LED_STRIP_SPI3_MOSI_IO_NUM (23)

#if HELPER_TARGET_IS_ESP32
#   if HELPER_TARGET_VERSION < HELPER_TARGET_VERSION_ESP32_V4
#       define LED_STRIP_DEFAULT_SPI_HOST  HSPI_HOST
#   else
#       define LED_STRIP_DEFAULT_SPI_HOST  SPI2_HOST
#   endif
#endif

#if HELPER_TARGET_IS_ESP8266
#define LED_STRIP_DEFAULT_SPI_HOST HSPI_HOST
#endif

#if !defined(LED_STRIP_DEFAULT_SPI_HOST)
#error "BUG: LED_STRIP_DEFAULT_SPI_HOST is not defined"
#endif

#if HELPER_TARGET_IS_ESP32
#define LED_STRIP_SPI_DEFAULT() \
{                                                       \
    .length = 1,                                        \
    .buf = NULL,                                        /* must be provided by the caller */ \
    .bus_config = {                                     \
        .miso_io_num = -1,                              \
        .mosi_io_num = LED_STRIP_SPI2_MOSI_IO_NUM,      \
        .sclk_io_num = LED_STRIP_SPI2_SCLK_IO_NUM,      \
        .quadhd_io_num = -1,                            \
        .quadwp_io_num = -1,                            \
        .max_transfer_sz = 0,                           /* must be provided by the caller */ \
    },                                                  \
    .device_interface_config = {                        \
        .clock_speed_hz = 1000000,                      \
        .mode = 3,                                      \
        .spics_io_num = -1,                             \
        .queue_size = 10,                               \
        .command_bits = 0,                              \
        .address_bits = 0,                              \
        .dummy_bits = 0,                                \
    },                                                  \
    .host_device = LED_STRIP_DEFAULT_SPI_HOST,          \
    .device_handle = NULL,                              /* must be provided by the caller */ \
    .dma_chan = 1,                                      \
}

#elif HELPER_TARGET_IS_ESP8266
#define LED_STRIP_SPI_DEFAULT() \
{                                                       \
    .length = 1,                                        \
    .buf = NULL,                                        /* must be provided by the caller */ \
    .bus_config = {                                     /* spi_interface_t */ \
        .cpol = 1,                                      /* SPI mode 3i; CPOL = 1, CPHA = 1 */\
        .cpha = 1,                                      \
        .bit_tx_order = 0,                              \
        .bit_rx_order = 0,                              \
        .byte_tx_order = 0,                             \
        .byte_rx_order = 0,                             \
        .mosi_en = 1,                                   \
        .miso_en = 0,                                   \
        .cs_en = 0,                                     \
        .reserved9 = 23,                                \
    },                                                  \
    .host_device = LED_STRIP_DEFAULT_SPI_HOST,          \
    .clk_div = SPI_2MHz_DIV,                            /* see components/esp8266/include/driver/spi.h */ \
}
#endif // HELPER_TARGET_IS_ESP32 HELPER_TARGET_IS_ESP8266

/*
 * * add LED_STRIP_SPI_USING_$NAME to Kconfig
 * * define `LED_STRIP_SPI_BUFFER_SIZE(N_PIXEL)` that returns the required
 *   size of buffer for the $NAME.
 */

#if defined(CONFIG_LED_STRIP_SPI_USING_SK9822)
#include "led_strip_spi_sk9822.h"
#else
#error "unknown LED type"
#endif

/**
 * @brief Setup library
 * This method must be called before any other led_strip methods
 */
esp_err_t led_strip_spi_install();

/**
 * @brief Initialize LED strip and allocate buffer memory
 * @param strip Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_init(led_strip_spi_t*strip);

/**
 * @brief Free LED strip
 * @param Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_free(led_strip_spi_t *strip);

/**
 * @brief Send strip buffer to LEDs
 * @param strip Descriptor of LED strip
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_flush(led_strip_spi_t*strip);

/**
 * @brief Check if associated RMT channel is busy
 * @param strip Descriptor of LED strip
 * @return true if RMT peripherals is busy
 */
bool led_strip_spi_busy(led_strip_spi_t*strip);

/**
 * @brief Wait until RMT peripherals is free to send buffer to LEDs
 * @param strip Descriptor of LED strip
 * @param timeout Timeout in RTOS ticks
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_wait(led_strip_spi_t*strip, TickType_t timeout);

/**
 * @brief Set color of single LED in strip
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_flush() to send buffer to the LEDs.
 * @param strip Descriptor of LED strip
 * @param num LED number, 0..strip length - 1
 * @param color RGB color
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_set_pixel(led_strip_spi_t *strip, size_t num, rgb_t color);

/**
 * @brief Set colors of multiple LEDs
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_flush() to send buffer to the LEDs.
 * @param strip Descriptor of LED strip
 * @param start First LED index, 0-based
 * @param len Number of LEDs
 * @param data Pointer to RGB data
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_set_pixels(led_strip_spi_t*strip, size_t start, size_t len, rgb_t *data);

/**
 * @brief Set multiple LEDs to the one color
 * This function does not actually change colors of the LEDs.
 * Call ::led_strip_flush() to send buffer to the LEDs.
 * @param strip Descriptor of LED strip
 * @param start First LED index, 0-based
 * @param len Number of LEDs
 * @param color RGB color
 * @return `ESP_OK` on success
 */
esp_err_t led_strip_spi_fill(led_strip_spi_t*strip, size_t start, size_t len, rgb_t color);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LED_STRIP_SPI_H__ */

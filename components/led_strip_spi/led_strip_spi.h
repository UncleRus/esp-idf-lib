/**
 * @file led_strip_spi.h
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

#if HELPER_TARGET_VERSION < HELPER_TARGET_VERSION_ESP32_V4
#define LED_STRIP_SPI_DEFAULT_HOST_DEVICE  HSPI_HOST
#else
#define LED_STRIP_SPI_DEFAULT_HOST_DEVICE  SPI2_HOST
#endif // HELPER_TARGET_VERSION < HELPER_TARGET_VERSION_ESP32_V4

#define LED_STRIP_SPI_DEFAULT_MOSI_IO_NUM   (13)
#define LED_STRIP_SPI_DEFAULT_SCLK_IO_NUM   (14)

typedef struct
{
    void *buf;                          /* Pointer to the buffer */
    size_t length;                      /* Number of pixels */
    spi_host_device_t host_device;      /* SPI host device name, such as `SPI2_HOST`. */
    int mosi_io_num;                    /* GPIO number of SPI MOSI. */
    int sclk_io_num;                    /* GPIO number of SPI SCLK. */
    int max_transfer_sz;                /* Maximum transfer size in bytes. Defaults to 4094 if 0. */
    int clock_speed_hz;                 /* Clock speed in Hz. */
    int queue_size;                     /* Queue size used by `spi_device_queue_trans()`. */
    spi_device_handle_t device_handle;  /* Pointer to device handle assigned by the driver. */
    int dma_chan;                       /* DMA channed to use. Either 1 or 2. */
    spi_transaction_t transaction;      /* SPI transaction used internally by the driver */
} led_strip_spi_t;

#define LED_STRIP_SPI_DEFAULT() \
{ \
    .buf = NULL,                                      \
    .length = 1,                                      \
    .host_device = LED_STRIP_SPI_DEFAULT_HOST_DEVICE, \
    .mosi_io_num = LED_STRIP_SPI_DEFAULT_MOSI_IO_NUM, \
    .sclk_io_num = LED_STRIP_SPI_DEFAULT_SCLK_IO_NUM, \
    .max_transfer_sz = 0,                             \
    .clock_speed_hz = 1000000,                        \
    .queue_size = 10,                                 \
    .device_handle = NULL,                            \
    .dma_chan = 1,                                    \
}

#elif HELPER_TARGET_IS_ESP8266
typedef struct
{
    void *buf;              /* Pointer to the buffer. */
    size_t length;          /* Number of pixels. */
    spi_clk_div_t clk_div;  /* Value of `clk_div`, such as `SPI_2MHz_DIV`. See available values in `${IDF_PATH}/components/esp8266/include/driver/spi.h`. */
} led_strip_spi_t;

#define LED_STRIP_SPI_DEFAULT() \
{ \
    .length = 1, \
    .clk_div = SPI_2MHz_DIV, \
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
 * This method must be called before any other led_strip_spi methods
 * @return `ESP_OK` on success
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
 * Call ::led_strip_spi_flush() to send buffer to the LEDs.
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

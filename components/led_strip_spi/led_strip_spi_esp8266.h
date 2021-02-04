/*
 * Copyright (C) 2021 Tomoyuki Sakurai <y@trombik.org>
 * MIT Licensed as described in the file LICENSE
 */

#if !defined(__LED_STRIP_SPI_ESP8266__H__)
#define __LED_STRIP_SPI_ESP8266__H__

/**
 * @file led_strip_spi_esp8266.h
 * @defgroup led_strip_spi_esp8266 led_strip_spi_esp8266
 *
 * @{
 */

#include <driver/spi.h>
#include "led_strip_spi_esp8266.h"

/**
 * @struct led_strip_spi_esp8266_t
 *
 * LED strip descriptor for ESP8266.
 */
typedef struct
{
    void *buf;              ///< Pointer to the buffer.
    size_t length;          ///< Number of pixels.
    spi_clk_div_t clk_div;  ///< Value of `clk_div`, such as `SPI_2MHz_DIV`. See available values in `${IDF_PATH}/components/esp8266/include/driver/spi.h`.
} led_strip_spi_esp8266_t;

/**
 * @brief A macro to initialize led_strip_spi_esp8266_t.
 *
 * `length`: 1 `clk_div`: SPI_2MHz_DIV
 */
#define LED_STRIP_SPI_DEFAULT_ESP8266() \
{ \
    .length = 1, \
    .clk_div = SPI_2MHz_DIV, \
}

/** @} */

#endif // __LED_STRIP_SPI_ESP8266__H__

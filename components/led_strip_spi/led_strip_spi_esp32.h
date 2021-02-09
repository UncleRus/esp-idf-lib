/*
 * MIT License
 *
 * Copyright (C) 2021 Tomoyuki Sakurai <y@rombik.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file led_strip_spi_esp8266.h
 * @defgroup led_strip_spi_esp32 led_strip_spi_esp32
 * @{
 */

#if !defined(__LED_STRIP_SPI_ESP32__H__)
#define __LED_STRIP_SPI_ESP32__H__

#include <esp_idf_lib_helpers.h>
#include <driver/spi_master.h>

#if HELPER_TARGET_VERSION < HELPER_TARGET_VERSION_ESP32_V4
#define LED_STRIP_SPI_DEFAULT_HOST_DEVICE  HSPI_HOST
#else
#define LED_STRIP_SPI_DEFAULT_HOST_DEVICE  SPI2_HOST ///< Default is `HSPI_HOST` (`SPI2_HOST` if `esp-idf` version is v3.x).
#endif // HELPER_TARGET_VERSION HELPER_TARGET_VERSION_ESP32_V4

#define LED_STRIP_SPI_DEFAULT_MOSI_IO_NUM   (13) ///< GPIO pin number of `LED_STRIP_SPI_DEFAULT_HOST_DEVICE`'s MOSI (13)
#define LED_STRIP_SPI_DEFAULT_SCLK_IO_NUM   (14) ///< GPIO pin number of `LED_STRIP_SPI_DEFAULT_HOST_DEVICE`'s SCLK (14)

/**
 * LED strip descriptor for ESP32-family.
 */
typedef struct
{
    void *buf;                          //< Pointer to the buffer
    size_t length;                      //< Number of pixels
    spi_host_device_t host_device;      //< SPI host device name, such as `SPI2_HOST`.
    int mosi_io_num;                    ///< GPIO number of SPI MOSI.
    int sclk_io_num;                    ///< GPIO number of SPI SCLK.
    int max_transfer_sz;                ///< Maximum transfer size in bytes. Defaults to 4094 if 0.
    int clock_speed_hz;                 ///< Clock speed in Hz.
    int queue_size;                     ///< Queue size used by `spi_device_queue_trans()`.
    spi_device_handle_t device_handle;  ///< Device handle assigned by the driver. The caller must provdie this.
    int dma_chan;                       ///< DMA channed to use. Either 1 or 2.
    spi_transaction_t transaction;      ///< SPI transaction used internally by the driver.
} led_strip_spi_esp32_t;

/**
 * Macro to initialize ::led_strip_spi_esp32_t
 *
 * `buf`: `NULL`,
 * `length`: 1,
 * `host_device`: `LED_STRIP_SPI_DEFAULT_HOST_DEVICE`,
 * `mosi_io_num`: `LED_STRIP_SPI_DEFAULT_MOSI_IO_NUM`,
 * `max_transfer_sz`: 0,
 * `clock_speed_hz`: 1000000,
 * `queue_size`: 1,
 * `device_handle`: `NULL`,
 * `dma_chan`: 1
 */
#define LED_STRIP_SPI_DEFAULT_ESP32() \
{ \
    .buf = NULL,                                      \
    .length = 1,                                      \
    .host_device = LED_STRIP_SPI_DEFAULT_HOST_DEVICE, \
    .mosi_io_num = LED_STRIP_SPI_DEFAULT_MOSI_IO_NUM, \
    .sclk_io_num = LED_STRIP_SPI_DEFAULT_SCLK_IO_NUM, \
    .max_transfer_sz = 0,                             \
    .clock_speed_hz = 1000000,                        \
    .queue_size = 1,                                  \
    .device_handle = NULL,                            \
    .dma_chan = 1,                                    \
}

/** @} */

#endif // __LED_STRIP_SPI_ESP32__H__

/*
 * Copyright (c) 2020 Erriez <https://github.com/Erriez>
 * Copyright (c) 2021 David Douard <david.douard@sdfa3.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mhz19b.c
 *
 * ESP-IDF driver for MH-Z19B NDIR CO2 sensor connected to UART
 *
 * Inspired from https://github.com/Erriez/ErriezMHZ19B
 *
 * Copyright (c) 2020 Erriez <https://github.com/Erriez>
 * Copyright (c) 2021 David Douard <david.douard@sdfa3.org>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <string.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "mhz19b.h"

static const char *TAG = "mhz19b";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t mhz19b_init(mhz19b_dev_t *dev, uart_port_t uart_port, gpio_num_t tx_gpio, gpio_num_t rx_gpio)
{
    CHECK_ARG(dev);

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
        .source_clk = UART_SCLK_APB,
#endif
    };
    CHECK(uart_driver_install(uart_port, MHZ19B_SERIAL_BUF_LEN * 2, 0, 0, NULL, 0));
    CHECK(uart_param_config(uart_port, &uart_config));
#if HELPER_TARGET_IS_ESP32
    CHECK(uart_set_pin(uart_port, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    dev->uart_port = uart_port;
    // buffer for the incoming data
    dev->buf = malloc(MHZ19B_SERIAL_BUF_LEN);
    if (!dev->buf)
        return ESP_ERR_NO_MEM;
    dev->last_value = -1;
    dev->last_ts = esp_timer_get_time();
    return ESP_OK;
}

esp_err_t mhz19b_free(mhz19b_dev_t *dev)
{
    CHECK_ARG(dev && dev->buf);

    free(dev->buf);
    dev->buf = NULL;
    return ESP_OK;
}

bool mhz19b_detect(mhz19b_dev_t *dev)
{
    CHECK_ARG(dev);

    uint16_t range;
    // Check valid PPM range
    if ((mhz19b_get_range(dev, &range) == ESP_OK) && (range > 0))
        return true;

    // Sensor not detected, or invalid range returned
    // Try recover by calling setRange(MHZ19B_RANGE_5000);
    return false;
}

bool mhz19b_is_warming_up(mhz19b_dev_t *dev, bool smart_warming_up)
{
    CHECK_ARG(dev);

    // Wait at least 3 minutes after power-on
    if (esp_timer_get_time() < MHZ19B_WARMING_UP_TIME_US)
    {
        if (smart_warming_up)
        {
            ESP_LOGD(TAG, "Using smart warming up detection");

            int16_t co2, last_co2;
            last_co2 = dev->last_value;
            // Sensor returns valid data after CPU reset and keep sensor powered
            if (mhz19b_read_co2(dev, &co2) != ESP_OK)
                return false;
            if ((last_co2 != -1) && (last_co2 != co2))
                // CO2 value changed since last read, no longer warming-up
                return false;
        }
        // Warming-up
        return true;
    }

    // Not warming-up
    return false;
}

bool mhz19b_is_ready(mhz19b_dev_t *dev)
{
    if (!dev) return false;

    // Minimum CO2 read interval (Built-in LED flashes)
    if ((esp_timer_get_time() - dev->last_ts) > MHZ19B_READ_INTERVAL_MS) {
        return true;
    }

    return false;
}

esp_err_t mhz19b_read_co2(mhz19b_dev_t *dev, int16_t *co2)
{
    CHECK_ARG(dev && co2);

    // Send command "Read CO2 concentration"
    CHECK(mhz19b_send_command(dev, MHZ19B_CMD_READ_CO2, 0, 0, 0, 0, 0));

    // 16-bit CO2 value in response Bytes 2 and 3
    *co2 = (dev->buf[2] << 8) | dev->buf[3];
    dev->last_ts = esp_timer_get_time();
    dev->last_value = *co2;

    return ESP_OK;
}

esp_err_t mhz19b_get_version(mhz19b_dev_t *dev, char *version)
{
    CHECK_ARG(dev && version);

    // Clear version
    memset(version, 0, 5);

    // Send command "Read firmware version" (NOT DOCUMENTED)
    CHECK(mhz19b_send_command(dev, MHZ19B_CMD_GET_VERSION, 0, 0, 0, 0, 0));

    // Copy 4 ASCII characters to version array like "0443"
    for (uint8_t i = 0; i < 4; i++) {
        // Version in response Bytes 2..5
        version[i] = dev->buf[i + 2];
    }

    return ESP_OK;
}

esp_err_t mhz19b_set_range(mhz19b_dev_t *dev, mhz19b_range_t range)
{
    CHECK_ARG(dev);

    // Send "Set range" command
    return mhz19b_send_command(dev, MHZ19B_CMD_SET_RANGE,
                               0x00, 0x00, 0x00, (range >> 8), (range & 0xff));
}

esp_err_t mhz19b_get_range(mhz19b_dev_t *dev, uint16_t *range)
{
    CHECK_ARG(dev && range);

    // Send command "Read range" (NOT DOCUMENTED)
    CHECK(mhz19b_send_command(dev, MHZ19B_CMD_GET_RANGE, 0, 0, 0, 0, 0));

    // Range is in Bytes 4 and 5
    *range = (dev->buf[4] << 8) | dev->buf[5];

    // Check range according to documented specification
    if ((*range != MHZ19B_RANGE_2000) && (*range != MHZ19B_RANGE_5000))
        return ESP_ERR_INVALID_RESPONSE;

    return ESP_OK;
}

esp_err_t mhz19b_set_auto_calibration(mhz19b_dev_t *dev, bool calibration_on)
{
    CHECK_ARG(dev);

    // Send command "Set Automatic Baseline Correction (ABC logic function)"
    return mhz19b_send_command(dev, MHZ19B_CMD_SET_AUTO_CAL, (calibration_on ? 0xA0 : 0x00), 0, 0, 0, 0);
}

esp_err_t mhz19b_get_auto_calibration(mhz19b_dev_t *dev, bool *calibration_on)
{
    CHECK_ARG(dev && calibration_on);

    // Send command "Get Automatic Baseline Correction (ABC logic function)" (NOT DOCUMENTED)
    CHECK(mhz19b_send_command(dev, MHZ19B_CMD_GET_AUTO_CAL, 0, 0, 0, 0, 0));

    // Response is located in Byte 7: 0 = off, 1 = on
    *calibration_on = dev->buf[7] & 0x01;

    return ESP_OK;
}

esp_err_t mhz19b_start_calibration(mhz19b_dev_t *dev)
{
    CHECK_ARG(dev);

    // Send command "Zero Point Calibration"
    return mhz19b_send_command(dev, MHZ19B_CMD_CAL_ZERO_POINT, 0, 0, 0, 0, 0);
}

esp_err_t mhz19b_send_command(mhz19b_dev_t *dev, uint8_t cmd, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
    CHECK_ARG(dev && dev->buf);

    uint8_t txBuffer[MHZ19B_SERIAL_RX_BYTES] = { 0xFF, 0x01, cmd, b3, b4, b5, b6, b7, 0x00 };

    // Check initialized
#if HELPER_TARGET_IS_ESP32 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    if (!uart_is_driver_installed(dev->uart_port))
        return ESP_ERR_INVALID_STATE;
#endif

    // Add CRC Byte
    txBuffer[8] = mhz19b_calc_crc(txBuffer);

    // Clear receive buffer
    uart_flush(dev->uart_port);

    // Write serial data
    uart_write_bytes(dev->uart_port, (char *) txBuffer, sizeof(txBuffer));

    // Clear receive buffer
    memset(dev->buf, 0, MHZ19B_SERIAL_BUF_LEN);

    // Read response from serial buffer
    int len = uart_read_bytes(dev->uart_port, dev->buf,
                              MHZ19B_SERIAL_RX_BYTES,
                              MHZ19B_SERIAL_RX_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (len < 9)
        return ESP_ERR_TIMEOUT;

    // Check received Byte[0] == 0xFF and Byte[1] == transmit command
    if ((dev->buf[0] != 0xFF) || (dev->buf[1] != cmd))
        return ESP_ERR_INVALID_RESPONSE;

    // Check received Byte[8] CRC
    if (dev->buf[8] != mhz19b_calc_crc(dev->buf))
        return ESP_ERR_INVALID_CRC;

    // Return result
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------

uint8_t mhz19b_calc_crc(uint8_t *data)
{
    uint8_t crc = 0;

    // Calculate CRC on 8 data Bytes
    for (uint8_t i = 1; i < 8; i++)
        crc += data[i];

    crc = 0xFF - crc;
    crc++;

    // Return calculated CRC
    return crc;
}

/*
 * Copyright (c) 2016 Grzegorz Hetman <ghetman@gmail.com>
 * Copyright (c) 2016 Alex Stewart <foogod@gmail.com>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file ds18x20.c
 *
 * ESP-IDF driver for the DS18S20/DS18B20 one-wire temperature sensor ICs
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Grzegorz Hetman <ghetman@gmail.com>\n
 * Copyright (c) 2016 Alex Stewart <foogod@gmail.com>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include "ds18x20.h"

#define ds18x20_WRITE_SCRATCHPAD 0x4E
#define ds18x20_READ_SCRATCHPAD  0xBE
#define ds18x20_COPY_SCRATCHPAD  0x48
#define ds18x20_READ_EEPROM      0xB8
#define ds18x20_READ_PWRSUPPLY   0xB4
#define ds18x20_SEARCHROM        0xF0
#define ds18x20_SKIP_ROM         0xCC
#define ds18x20_READROM          0x33
#define ds18x20_MATCHROM         0x55
#define ds18x20_ALARMSEARCH      0xEC
#define ds18x20_CONVERT_T        0x44

#define SLEEP_MS(x) vTaskDelay(((x) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#if HELPER_TARGET_IS_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

#elif HELPER_TARGET_IS_ESP8266
#define PORT_ENTER_CRITICAL portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL()
#endif

static const char *TAG = "ds18x20";

esp_err_t ds18x20_measure(gpio_num_t pin, onewire_addr_t addr, bool wait)
{
    if (!onewire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == DS18X20_ANY)
        onewire_skip_rom(pin);
    else
        onewire_select(pin, addr);

    PORT_ENTER_CRITICAL;
    onewire_write(pin, ds18x20_CONVERT_T);
    // For parasitic devices, power must be applied within 10us after issuing
    // the convert command.
    onewire_power(pin);
    PORT_EXIT_CRITICAL;

    if (wait)
    {
        SLEEP_MS(750);
        onewire_depower(pin);
    }

    return ESP_OK;
}

esp_err_t ds18x20_read_scratchpad(gpio_num_t pin, onewire_addr_t addr, uint8_t *buffer)
{
    CHECK_ARG(buffer);

    uint8_t crc;
    uint8_t expected_crc;

    if (!onewire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == DS18X20_ANY)
        onewire_skip_rom(pin);
    else
        onewire_select(pin, addr);
    onewire_write(pin, ds18x20_READ_SCRATCHPAD);

    for (int i = 0; i < 8; i++)
        buffer[i] = onewire_read(pin);
    crc = onewire_read(pin);

    expected_crc = onewire_crc8(buffer, 8);
    if (crc != expected_crc)
    {
        ESP_LOGE(TAG, "CRC check failed reading scratchpad: %02x %02x %02x %02x %02x %02x %02x %02x : %02x (expected %02x)", buffer[0], buffer[1],
                buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], crc, expected_crc);
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

esp_err_t ds18x20_write_scratchpad(gpio_num_t pin, onewire_addr_t addr, uint8_t *buffer)
{
    CHECK_ARG(buffer);

    if (!onewire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == DS18X20_ANY)
        onewire_skip_rom(pin);
    else
        onewire_select(pin, addr);
    onewire_write(pin, ds18x20_WRITE_SCRATCHPAD);

    for (int i = 0; i < 3; i++)
        onewire_write(pin, buffer[i]);

    return ESP_OK;
}

esp_err_t ds18x20_copy_scratchpad(gpio_num_t pin, onewire_addr_t addr)
{
    if (!onewire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == DS18X20_ANY)
        onewire_skip_rom(pin);
    else
        onewire_select(pin, addr);

    PORT_ENTER_CRITICAL;
    onewire_write(pin, ds18x20_COPY_SCRATCHPAD);
    // For parasitic devices, power must be applied within 10us after issuing
    // the convert command.
    onewire_power(pin);
    PORT_EXIT_CRITICAL;

    // And then it needs to keep that power up for 10ms.
    SLEEP_MS(10);
    onewire_depower(pin);

    return ESP_OK;
}

esp_err_t ds18s20_read_temperature(gpio_num_t pin, onewire_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    uint8_t scratchpad[8];
    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    int16_t temp = (((scratchpad[1] << 8) | (scratchpad[0] & 0xfe)) << 3) | ((0x10 - scratchpad[6]) & 0x0f);
    *temperature = (float)temp * 0.0625f - 0.250f;

    return ESP_OK;
}

esp_err_t ds18b20_read_temperature(gpio_num_t pin, onewire_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    uint8_t scratchpad[8];
    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    uint16_t temp = scratchpad[1] << 8 | scratchpad[0];
    int sign = 1;
    if (temp > 2047)
    {
        temp = ~temp + 1;
        sign = -1;
    }
    *temperature = (float)temp * (float)sign * 0.0625f;

    return ESP_OK;
}

esp_err_t max31850_read_temperature(gpio_num_t pin, onewire_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    uint8_t scratchpad[8];
    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    int16_t temp = scratchpad[1] << 8 | (scratchpad[0] & 0xfc);
    *temperature = (float)temp * 0.0625f;

    return ESP_OK;
}

esp_err_t ds18x20_read_temperature(gpio_num_t pin, onewire_addr_t addr, float *temperature)
{
    uint8_t family = (uint8_t)addr;
    switch (family)
    {
        case DS18X20_FAMILY_DS18S20:
        case DS18X20_FAMILY_DS1822:
            return ds18s20_read_temperature(pin, addr, temperature);
        case DS18X20_FAMILY_DS18B20:
            return ds18b20_read_temperature(pin, addr, temperature);
        case DS18X20_FAMILY_MAX31850:
            return max31850_read_temperature(pin, addr, temperature);
        default:
            ESP_LOGE(TAG, "Unknown sensor family %02x. Please use an explicit read/measure function", family);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ds18b20_measure_and_read(gpio_num_t pin, onewire_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return ds18b20_read_temperature(pin, addr, temperature);
}

esp_err_t ds18s20_measure_and_read(gpio_num_t pin, onewire_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return ds18s20_read_temperature(pin, addr, temperature);
}

esp_err_t max31850_measure_and_read(gpio_num_t pin, onewire_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return max31850_read_temperature(pin, addr, temperature);
}

esp_err_t ds18x20_measure_and_read(gpio_num_t pin, onewire_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return ds18x20_read_temperature(pin, addr, temperature);
}

esp_err_t ds18x20_measure_and_read_multi(gpio_num_t pin, onewire_addr_t *addr_list, size_t addr_count, float *result_list)
{
    CHECK_ARG(result_list && addr_count);

    CHECK(ds18x20_measure(pin, DS18X20_ANY, true));

    return ds18x20_read_temp_multi(pin, addr_list, addr_count, result_list);
}

esp_err_t ds18x20_scan_devices(gpio_num_t pin, onewire_addr_t *addr_list, size_t addr_count, size_t *found)
{
    CHECK_ARG(addr_list && addr_count);

    onewire_search_t search;
    onewire_addr_t addr;

    *found = 0;
    onewire_search_start(&search);
    while ((addr = onewire_search_next(&search, pin)) != ONEWIRE_NONE)
    {
        uint8_t family_id = (uint8_t)addr;
        if (family_id == DS18X20_FAMILY_DS18S20
            || family_id == DS18X20_FAMILY_DS1822
            || family_id == DS18X20_FAMILY_DS18B20
            || family_id == DS18X20_FAMILY_MAX31850)
        {
            if (*found < addr_count)
                addr_list[*found] = addr;
            *found += 1;
        }
    }

    return ESP_OK;
}

esp_err_t ds18x20_read_temp_multi(gpio_num_t pin, onewire_addr_t *addr_list, size_t addr_count, float *result_list)
{
    CHECK_ARG(result_list);

    esp_err_t res = ESP_OK;
    for (size_t i = 0; i < addr_count; i++)
    {
        esp_err_t tmp = ds18x20_read_temperature(pin, addr_list[i], &result_list[i]);
        if (tmp != ESP_OK)
            res = tmp;
    }
    return res;
}

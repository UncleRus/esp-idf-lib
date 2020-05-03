/**
 * @file ds18x20.c
 *
 * ESP-IDF driver for the DS18S20/DS18B20 one-wire temperature sensor ICs
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016 Grzegorz Hetman <ghetman@gmail.com>\n
 * Copyright (C) 2016 Alex Stewart <foogod@gmail.com>\n
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <math.h>
#include <esp_log.h>
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

#define DS18B20_FAMILY_ID 0x28
#define DS18S20_FAMILY_ID 0x10

#if HELPER_TARGET_IS_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

#elif HELPER_TARGET_IS_ESP8266
#define PORT_ENTER_CRITICAL portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL()
#endif

static const char *TAG = "DS18x20";

esp_err_t ds18x20_measure(gpio_num_t pin, ds18x20_addr_t addr, bool wait)
{
    if (!onewire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == ds18x20_ANY)
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

esp_err_t ds18x20_read_scratchpad(gpio_num_t pin, ds18x20_addr_t addr, uint8_t *buffer)
{
    CHECK_ARG(buffer);

    uint8_t crc;
    uint8_t expected_crc;

    if (!onewire_reset(pin))
        return ESP_ERR_INVALID_RESPONSE;

    if (addr == ds18x20_ANY)
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

esp_err_t ds18x20_read_temperature(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    uint8_t scratchpad[8];
    int16_t temp;

    CHECK(ds18x20_read_scratchpad(pin, addr, scratchpad));

    temp = scratchpad[1] << 8 | scratchpad[0];

    if ((uint8_t)addr == DS18B20_FAMILY_ID)
        *temperature = ((float)temp * 625.0) / 10000;
    else
    {
        temp = ((temp & 0xfffe) << 3) + (16 - scratchpad[6]) - 4;
        *temperature = ((float)temp * 625.0) / 10000 - 0.25;
    }

    return ESP_OK;
}

esp_err_t ds18x20_measure_and_read(gpio_num_t pin, ds18x20_addr_t addr, float *temperature)
{
    CHECK_ARG(temperature);

    CHECK(ds18x20_measure(pin, addr, true));
    return ds18x20_read_temperature(pin, addr, temperature);
}

esp_err_t ds18x20_measure_and_read_multi(gpio_num_t pin, ds18x20_addr_t *addr_list, int addr_count, float *result_list)
{
    CHECK_ARG(result_list);

    CHECK(ds18x20_measure(pin, ds18x20_ANY, true));

    return ds18x20_read_temp_multi(pin, addr_list, addr_count, result_list);
}

int ds18x20_scan_devices(gpio_num_t pin, ds18x20_addr_t *addr_list, int addr_count)
{
    CHECK_ARG(addr_list);

    onewire_search_t search;
    onewire_addr_t addr;
    int found = 0;

    onewire_search_start(&search);
    while ((addr = onewire_search_next(&search, pin)) != ONEWIRE_NONE)
    {
        uint8_t family_id = (uint8_t)addr;
        if (family_id == DS18B20_FAMILY_ID || family_id == DS18S20_FAMILY_ID)
        {
            if (found < addr_count)
                addr_list[found] = addr;
            found++;
        }
    }
    return found;
}

esp_err_t ds18x20_read_temp_multi(gpio_num_t pin, ds18x20_addr_t *addr_list, int addr_count, float *result_list)
{
    CHECK_ARG(result_list);

    esp_err_t res = ESP_OK;
    for (int i = 0; i < addr_count; i++)
    {
        esp_err_t tmp = ds18x20_read_temperature(pin, addr_list[i], &result_list[i]);
        if (tmp != ESP_OK)
            res = tmp;
    }
    return res;
}


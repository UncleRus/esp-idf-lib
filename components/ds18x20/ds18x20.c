#include "ds18x20.h"

#include <math.h>
#include <esp_log.h>

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

#define os_sleep_ms(x) vTaskDelay(((x) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)

#define DS18B20_FAMILY_ID 0x28
#define DS18S20_FAMILY_ID 0x10

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static const char *TAG = "ds18x20";

bool ds18x20_measure(gpio_num_t pin, ds18x20_addr_t addr, bool wait)
{
    if (!onewire_reset(pin))
        return false;
    if (addr == ds18x20_ANY)
        onewire_skip_rom(pin);
    else
        onewire_select(pin, addr);

    taskENTER_CRITICAL(&mux);
    onewire_write(pin, ds18x20_CONVERT_T);
    // For parasitic devices, power must be applied within 10us after issuing
    // the convert command.
    onewire_power(pin);
    taskEXIT_CRITICAL(&mux);

    if (wait)
    {
        os_sleep_ms(750);
        onewire_depower(pin);
    }

    return true;
}

bool ds18x20_read_scratchpad(gpio_num_t pin, ds18x20_addr_t addr, uint8_t *buffer)
{
    uint8_t crc;
    uint8_t expected_crc;

    if (!onewire_reset(pin))
        return false;
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
        return false;
    }

    return true;
}

float ds18x20_read_temperature(gpio_num_t pin, ds18x20_addr_t addr)
{
    uint8_t scratchpad[8];
    int16_t temp;

    if (!ds18x20_read_scratchpad(pin, addr, scratchpad))
        return NAN;

    temp = scratchpad[1] << 8 | scratchpad[0];

    float res;
    if ((uint8_t)addr == DS18B20_FAMILY_ID)
        res = ((float)temp * 625.0) / 10000;
    else
    {
        temp = ((temp & 0xfffe) << 3) + (16 - scratchpad[6]) - 4;
        res = ((float)temp * 625.0) / 10000 - 0.25;
    }
    return res;
}

float ds18x20_measure_and_read(gpio_num_t pin, ds18x20_addr_t addr)
{
    if (!ds18x20_measure(pin, addr, true))
        return NAN;
    return ds18x20_read_temperature(pin, addr);
}

bool ds18x20_measure_and_read_multi(gpio_num_t pin, ds18x20_addr_t *addr_list, int addr_count, float *result_list)
{
    if (!ds18x20_measure(pin, ds18x20_ANY, true))
    {
        for (int i = 0; i < addr_count; i++)
            result_list[i] = NAN;
        return false;
    }
    return ds18x20_read_temp_multi(pin, addr_list, addr_count, result_list);
}

int ds18x20_scan_devices(gpio_num_t pin, ds18x20_addr_t *addr_list, int addr_count)
{
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

bool ds18x20_read_temp_multi(gpio_num_t pin, ds18x20_addr_t *addr_list, int addr_count, float *result_list)
{
    bool result = true;

    for (int i = 0; i < addr_count; i++)
    {
        result_list[i] = ds18x20_read_temperature(pin, addr_list[i]);
        if (isnan(result_list[i]))
            result = false;
    }
    return result;
}


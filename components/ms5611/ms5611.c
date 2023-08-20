/*
 * Copyright (c) 2016 Bernhard Guillon <Bernhard.Guillon@begu.org>
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
 * @file ms5611.c
 *
 * ESP-IDF driver for barometric pressure sensor MS5611-01BA03
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Bernhard Guillon <Bernhard.Guillon@begu.org>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_idf_lib_helpers.h>
#include "ms5611.h"

#include <stddef.h>
#include <esp_system.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ets_sys.h>

#define I2C_FREQ_HZ 400000 // 400 kHz

#define CMD_CONVERT_D1 0x40
#define CMD_CONVERT_D2 0x50
#define CMD_ADC_READ   0x00
#define CMD_RESET      0x1E

#define PROM_ADDR_SENS     0xa2
#define PROM_ADDR_OFF      0xa4
#define PROM_ADDR_TCS      0xa6
#define PROM_ADDR_TCO      0xa8
#define PROM_ADDR_T_REF    0xaa
#define PROM_ADDR_TEMPSENS 0xac

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "ms5611";

static inline esp_err_t send_command(ms5611_t *dev, uint8_t cmd)
{
    return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1);
}

static inline uint16_t shuffle(uint16_t val)
{
    return ((val & 0xff00) >> 8) | ((val & 0xff) << 8);
}

static inline esp_err_t read_prom(ms5611_t *dev)
{
    uint16_t tmp;

    // FIXME calculate CRC (AN502)

    CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_SENS, &tmp, 2));
    dev->config_data.sens = shuffle(tmp);
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_OFF, &tmp, 2));
    dev->config_data.off = shuffle(tmp);
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_TCS, &tmp, 2));
    dev->config_data.tcs = shuffle(tmp);
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_TCO, &tmp, 2));
    dev->config_data.tco = shuffle(tmp);
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_T_REF, &tmp, 2));
    dev->config_data.t_ref = shuffle(tmp);
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, PROM_ADDR_TEMPSENS, &tmp, 2));
    dev->config_data.tempsens = shuffle(tmp);

    return ESP_OK;
}

static esp_err_t read_adc(ms5611_t *dev, uint32_t *result)
{
    uint8_t tmp[3];

    CHECK(i2c_dev_read_reg(&dev->i2c_dev, 0, tmp, 3));
    *result = (tmp[0] << 16) | (tmp[1] << 8) | tmp[2];

    // If we are to fast the ADC will return 0 instead of the actual result
    return *result == 0 ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}

static void wait_conversion(ms5611_t *dev)
{
    uint32_t us = 8220;
    switch (dev->osr)
    {
        case MS5611_OSR_256: us = 500; break;   // 0.5ms
        case MS5611_OSR_512: us = 1100; break;  // 1.1ms
        case MS5611_OSR_1024: us = 2100; break; // 2.1ms
        case MS5611_OSR_2048: us = 4100; break; // 4.1ms
        case MS5611_OSR_4096: us = 8220; break; // 8.22ms
    }
    ets_delay_us(us);
}

static inline esp_err_t get_raw_temperature(ms5611_t *dev, uint32_t *result)
{
    CHECK(send_command(dev, CMD_CONVERT_D2 + dev->osr));
    wait_conversion(dev);
    CHECK(read_adc(dev, result));

    return ESP_OK;
}

static inline esp_err_t get_raw_pressure(ms5611_t *dev, uint32_t *result)
{
    CHECK(send_command(dev, CMD_CONVERT_D1 + dev->osr));
    wait_conversion(dev);
    CHECK(read_adc(dev, result));

    return ESP_OK;
}

static esp_err_t ms5611_reset(ms5611_t *dev)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_command(dev, CMD_RESET));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

/////////////////////////Public//////////////////////////////////////

esp_err_t ms5611_init_desc(ms5611_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != MS5611_ADDR_CSB_HIGH && addr != MS5611_ADDR_CSB_LOW)
    {
        ESP_LOGE(TAG, "Invalid I2C address 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t ms5611_free_desc(ms5611_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t ms5611_init(ms5611_t *dev, ms5611_osr_t osr)
{
    CHECK_ARG(dev);

    dev->osr = osr;

    // First of all we need to reset the chip
    CHECK(ms5611_reset(dev));
    // Wait a bit for the device to reset
    vTaskDelay(pdMS_TO_TICKS(10));
    // Get the config
    CHECK(read_prom(dev));

    return ESP_OK;
}

esp_err_t ms5611_get_sensor_data(ms5611_t *dev, int32_t *pressure, float *temperature)
{
    CHECK_ARG(dev && pressure && temperature);

    // Second order temperature compensation see datasheet p8
    uint32_t raw_pressure = 0;
    uint32_t raw_temperature = 0;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, get_raw_pressure(dev, &raw_pressure));
    I2C_DEV_CHECK(&dev->i2c_dev, get_raw_temperature(dev, &raw_temperature));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // dT = D2 - T_ref = D2 - C5 * 2^8
    int32_t dt = raw_temperature - ((int32_t)dev->config_data.t_ref << 8);
    // Actual temperature (-40...85C with 0.01 resolution)
    // TEMP = 20C +dT * TEMPSENSE =2000 + dT * C6 / 2^23
    int64_t temp = (2000 + (int64_t)dt * dev->config_data.tempsens / 8388608);
    // Offset at actual temperature
    // OFF=OFF_t1 + TCO * dT = OFF_t1(C2) * 2^16 + (C4*dT)/2^7
    int64_t off = (int64_t)((int64_t)dev->config_data.off * 65536)
        + (((int64_t)dev->config_data.tco * dt) / 128);
    // Sensitivity at actual temperature
    // SENS=SENS_t1 + TCS *dT = SENS_t1(C1) *2^15 + (TCS(C3) *dT)/2^8
    int64_t sens = (int64_t)(((int64_t)dev->config_data.sens) * 32768)
        + (((int64_t)dev->config_data.tcs * dt) / 256);

    // Set defaults for temp >= 2000
    int64_t t_2 = 0;
    int64_t off_2 = 0;
    int64_t sens_2 = 0;
    int64_t help = 0;
    if (temp < 2000)
    {
        // Low temperature
        t_2 = ((dt * dt) >> 31); // T2 = dT^2/2^31
        help = (temp - 2000);
        help = 5 * help * help;
        off_2 = help >> 1;       // OFF_2  = 5 * (TEMP - 2000)^2/2^1
        sens_2 = help >> 2;      // SENS_2 = 5 * (TEMP - 2000)^2/2^2
        if (temp < -1500)
        {
            // Very low temperature
            help = (temp + 1500);
            help = help * help;
            off_2 = off_2 + 7 * help;             // OFF_2  = OFF_2 + 7 * (TEMP + 1500)^2
            sens_2 = sens_2 + ((11 * help) >> 1); // SENS_2 = SENS_2 + 7 * (TEMP + 1500)^2/2^1
        }
    }

    temp = temp - t_2;
    off = off - off_2;
    sens = sens - sens_2;

    // Temperature compensated pressure (10...1200mbar with 0.01mbar resolution
    // P = digital pressure value  * SENS - OFF = (D1 * SENS/2^21 -OFF)/2^15
    *pressure = (int32_t)(((int64_t)raw_pressure * (sens / 0x200000) - off) / 32768);
    *temperature = (float)temp / 100.0;

    return ESP_OK;
}

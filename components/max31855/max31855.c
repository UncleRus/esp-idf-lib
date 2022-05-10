/*
 * Copyright (c) 2022 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file max31855.c
 *
 * ESP-IDF driver for MAX31855 cold-junction compensated
 * thermocouple-to-digital converter
 *
 * Copyright (c) 2022 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "max31855.h"
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define BIT_TC_SIGN 31
#define BIT_TC      18
#define BIT_CJ_SIGN 15
#define BIT_CJ      4
#define BIT_SCV     2
#define BIT_SCG     1
#define BIT_OC      0

#define MASK_CJ 0x0fff
#define MASK_TC 0x3fff

#define SIGN_CJ 0xf000
#define SIGN_TC 0xc000

#define LSB_TC 0.25f
#define LSB_CJ 0.0625f

static inline esp_err_t read_32(max31855_t *dev, uint32_t *val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t rx[4];

    t.tx_buffer = NULL;
    t.rx_buffer = rx;
    t.length = 32;
    CHECK(spi_device_transmit(dev->spi_dev, &t));

    *val = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];

    return ESP_OK;
}

static inline esp_err_t max31855_get_raw_data(max31855_t *dev, int16_t *tc_t, int16_t *cj_t, bool *scv, bool *scg, bool *oc)
{
    CHECK_ARG(dev && tc_t && cj_t && scv && scg && oc);

    uint32_t v = 0;
    CHECK(read_32(dev, &v));

    // faults
    *scv = v & BIT(BIT_SCV) ? true : false;
    *scg = v & BIT(BIT_SCG) ? true : false;
    *oc = v & BIT(BIT_OC) ? true : false;

    // cold junction temperature
    *cj_t = (int16_t)((v >> BIT_CJ) & MASK_CJ) | (v & BIT(BIT_CJ_SIGN) ? SIGN_CJ : 0);

    // thermocouple temperature
    *tc_t = (int16_t)((v >> BIT_TC) & MASK_TC) | (v & BIT(BIT_TC_SIGN) ? SIGN_TC : 0);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t max31855_init_desc(max31855_t *dev, spi_host_device_t host, uint32_t clock_speed_hz, gpio_num_t cs_pin)
{
    CHECK_ARG(dev);

    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = cs_pin;
    dev->spi_cfg.clock_speed_hz = clock_speed_hz;
    dev->spi_cfg.mode = 0;
    dev->spi_cfg.queue_size = 1;
    dev->spi_cfg.cs_ena_pretrans = 1;

    return spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
}

esp_err_t max31855_free_desc(max31855_t *dev)
{
    CHECK_ARG(dev);

    return spi_bus_remove_device(dev->spi_dev);
}

esp_err_t max31855_get_temperature(max31855_t *dev, float *tc_t, float *cj_t, bool *scv, bool *scg, bool *oc)
{
    CHECK_ARG(tc_t);

    int16_t raw_tc = 0, raw_cj = 0;
    CHECK(max31855_get_raw_data(dev, &raw_tc, &raw_cj, scv, scg, oc));

    *tc_t = (float)raw_tc * LSB_TC;
    if (cj_t)
        *cj_t = (float)raw_cj * LSB_CJ;

    return ESP_OK;
}

/*
 * Copyright (c) 2022 Jaime Albuquerque <jaime.albq@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "shift_reg.h"

static char *tag = "shift_reg";

esp_err_t shift_reg_init(shift_reg_config_t *dev)
{
    esp_err_t err = ESP_FAIL;

    if (dev == NULL)
    {
        ESP_LOGE(tag, "%s: must have the configuration of the shift register", __func__);
        err = ESP_ERR_INVALID_ARG;
        return err;
    }

    dev->reg_value = (uint8_t *)malloc(dev->num_reg); // Create an array with all registers

    if (dev->reg_value == NULL)
    {
        ESP_LOGE(tag, "%s: no heap memory to allocate reg_value", __func__);
        err = ESP_ERR_NO_MEM;
        return err;
    }

    memset(dev->reg_value, 0, dev->num_reg); // Start all registers as 0

    gpio_config_t io_conf;

    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;

    switch (dev->mode.dir)
    {
        case SHIFT_DIR_OUTPUT:
            // set as output mode
            io_conf.mode = GPIO_MODE_OUTPUT;
            uint32_t buf32_0 = 0;
            uint32_t buf32_1 = 0;
            uint64_t result = 0;

            if (dev->pin.clk >= 32)
                buf32_1 |= 1 << (dev->pin.clk - 32);
            else
                buf32_0 |= 1 << dev->pin.clk;

            if (dev->pin.latch >= 32)
                buf32_1 |= 1 << (dev->pin.latch - 32);
            else
                buf32_0 |= 1 << dev->pin.latch;

            if (dev->pin.data >= 32)
                buf32_1 |= 1 << (dev->pin.data - 32);
            else
                buf32_0 |= 1 << dev->pin.data;

            result = ((uint64_t)buf32_1 << 32) | ((uint64_t)buf32_0 << 0);

            io_conf.pin_bit_mask = result;

            break;

        default:
            ESP_LOGE(tag, "%s: Mode of shift register not found", __func__);
            err = ESP_ERR_INVALID_ARG;
            break;
    }
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    err = gpio_config(&io_conf);

    return err;
}

esp_err_t shift_reg_deinit(shift_reg_config_t *dev)
{
    if (dev == NULL)
    {
        ESP_LOGE(tag, "%s: must have a valid argument;", __func__);
        return ESP_ERR_INVALID_ARG;
    }

    free(dev->reg_value);
    return ESP_OK;
}

esp_err_t shift_reg_send(shift_reg_config_t *dev, uint8_t *data, uint8_t len)
{
    esp_err_t err = ESP_FAIL;

    if (dev == NULL || len > dev->num_reg || data == NULL)
    {
        ESP_LOGE(tag, "%s: must have a valid argument;", __func__);
        err = ESP_ERR_INVALID_ARG;
        return err;
    }

    if (dev->mode.bit_mode == SHIFT_BIT_MODE_MSB)
    {
        for (uint8_t i = 0; i < len; i++)
        {
            shift_reg_send8bits(dev, data[i]);
            dev->reg_value[i] = data[i];
        }
    }
    else
    {
        for (int8_t i = len - 1; i >= 0; i--)
        {
            shift_reg_send8bits(dev, data[i]);
            dev->reg_value[i] = data[i];
        }
    }

    err = ESP_OK;

    return err;
}

esp_err_t shift_reg_send8bits(shift_reg_config_t *dev, uint8_t data)
{
    esp_err_t err = ESP_FAIL;

    if (dev == NULL)
    {
        ESP_LOGE(tag, "%s: must have a valid argument;", __func__);
        err = ESP_ERR_INVALID_ARG;
        return err;
    }

    if (dev->mode.bit_mode == SHIFT_BIT_MODE_MSB)
    {
        // MSB Mode
        for (int8_t i = 7; i >= 0; i--)
        {
            gpio_set_level(dev->pin.data, (data >> i) & 1);

            gpio_set_level(dev->pin.clk, true);
            ets_delay_us(1);
            gpio_set_level(dev->pin.clk, false);
            ets_delay_us(1);
        }
    }
    else
    {
        // LSB Mode
        for (int8_t i = 0; i < 8; i++)
        {
            gpio_set_level(dev->pin.data, (data >> i) & 1);

            gpio_set_level(dev->pin.clk, true);
            ets_delay_us(1);
            gpio_set_level(dev->pin.clk, false);
            ets_delay_us(1);
        }
    }

    err = ESP_OK;

    return err;
}

esp_err_t shift_reg_latch(shift_reg_config_t *dev)
{
    esp_err_t err = ESP_FAIL;

    if (dev == NULL)
    {
        ESP_LOGE(tag, "%s: must have a valid argument;", __func__);
        err = ESP_ERR_INVALID_ARG;
        return err;
    }

    gpio_set_level(dev->pin.latch, true);
    ets_delay_us(1);
    gpio_set_level(dev->pin.latch, false);
    ets_delay_us(1);

    err = ESP_OK;

    return err;
}

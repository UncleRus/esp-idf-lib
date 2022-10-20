/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Weslley M. F. Duarte <weslleymfd@gmail.com>
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
 * @file ads130e08.c
 *
 * ESP-IDF driver for ADS130E08 ADC
 *
 * Copyright (c) 2021 Weslley M. F. Duarte <weslleymfd@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "ads130e08.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#define ADS130E08_CMD_WAKEUP  (0x02)
#define ADS130E08_CMD_STANDBY (0x04)
#define ADS130E08_CMD_RESET   (0x06)
#define ADS130E08_CMD_START   (0x08)
#define ADS130E08_CMD_STOP    (0x0A)
#define ADS130E08_CMD_RDATAC  (0x10)
#define ADS130E08_CMD_SDATAC  (0x11)
#define ADS130E08_CMD_RDATA   (0x12)
#define ADS130E08_CMD_RREG    (0x20)
#define ADS130E08_CMD_WREG    (0x40)

#define ADS130E08_REG_ID          (0x00)
#define ADS130E08_REG_CONFIG1     (0x01)
#define ADS130E08_REG_CONFIG2     (0x02)
#define ADS130E08_REG_CONFIG3     (0x03)
#define ADS130E08_REG_FAULT       (0x04)
#define ADS130E08_REG_CH1SET      (0x05)
#define ADS130E08_REG_CH2SET      (0x06)
#define ADS130E08_REG_CH3SET      (0x07)
#define ADS130E08_REG_CH4SET      (0x08)
#define ADS130E08_REG_CH5SET      (0x09)
#define ADS130E08_REG_CH6SET      (0x0A)
#define ADS130E08_REG_CH7SET      (0x0B)
#define ADS130E08_REG_CH8SET      (0x0C)
#define ADS130E08_REG_FAULT_STATP (0x12)
#define ADS130E08_REG_FAULT_STATN (0x13)
#define ADS130E08_REG_GPIO        (0x14)

#define ID_MASK_LOW_BITS  (0x08)
#define ID_MASK_HIGH_BITS (0x10)

#define CONFIG1_MASK_LOW_BITS    (0xDE)
#define CONFIG1_MASK_HIGH_BITS   (0x01)
#define CONFIG1_MASK_BITS_CLK_EN BIT(5)

#define CONFIG2_MASK_LOW_BITS       (0x88)
#define CONFIG2_MASK_HIGH_BITS      (0x60)
#define CONFIG2_MASK_BITS_INT_TEST  BIT(4)
#define CONFIG2_MASK_BITS_TEST_AMP  BIT(2)
#define CONFIG2_MASK_BITS_TEST_FREQ (BIT(1) | BIT(0))

#define CONFIG3_MASK_LOW_BITS       (0x13)
#define CONFIG3_MASK_HIGH_BITS      (0x40)
#define CONFIG3_MASK_BITS_PD_REFBUF BIT(7)
#define CONFIG3_MASK_BITS_VREF_4V   BIT(5)
#define CONFIG3_MASK_BITS_OPAMP_REF BIT(3)
#define CONFIG3_MASK_BITS_PD_OPAMP  BIT(2)

#define FAULT_MASK_LOW_BITS     (0x1F)
#define FAULT_MASK_BITS_COMP_TH (BIT(7) | BIT(6) | BIT(5))

#define CHnSET_MASK_LOW_BITS (0x08)
#define CHnSET_MASK_BITS_PD  BIT(7)
#define CHnSET_MASK_BITS_PGA (BIT(6) | BIT(5) | BIT(4))
#define CHnSET_MASK_BITS_MUX (BIT(2) | BIT(1) | BIT(0))

#define MASK_BITS_IN1P_FAULT BIT(0)
#define MASK_BITS_IN2P_FAULT BIT(1)
#define MASK_BITS_IN3P_FAULT BIT(2)
#define MASK_BITS_IN4P_FAULT BIT(3)
#define MASK_BITS_IN5P_FAULT BIT(4)
#define MASK_BITS_IN6P_FAULT BIT(5)
#define MASK_BITS_IN7P_FAULT BIT(6)
#define MASK_BITS_IN8P_FAULT BIT(7)

#define MASK_BITS_IN1N_FAULT BIT(0)
#define MASK_BITS_IN2N_FAULT BIT(1)
#define MASK_BITS_IN3N_FAULT BIT(2)
#define MASK_BITS_IN4N_FAULT BIT(3)
#define MASK_BITS_IN5N_FAULT BIT(4)
#define MASK_BITS_IN6N_FAULT BIT(5)
#define MASK_BITS_IN7N_FAULT BIT(6)
#define MASK_BITS_IN8N_FAULT BIT(7)

#define MASK_BITS_GPIOC1 BIT(0)
#define MASK_BITS_GPIOC2 BIT(1)
#define MASK_BITS_GPIOC3 BIT(2)
#define MASK_BITS_GPIOC4 BIT(3)
#define MASK_BITS_GPIOD1 BIT(4)
#define MASK_BITS_GPIOD2 BIT(5)
#define MASK_BITS_GPIOD3 BIT(6)
#define MASK_BITS_GPIOD4 BIT(7)

#define CLOCK_SPEED_HZ (4096000) /**< 4MHz */

#define ADS130E08_CONVERSION_CONST 0.0000732421875f /* 2.4 / 32768 */

#define CHECK(x)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        esp_err_t __;                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                        \
            return __;                                                                                                 \
    }                                                                                                                  \
    while (0)
#define CHECK_ARG(VAL)                                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(VAL))                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                \
    }                                                                                                                  \
    while (0)
#define BV(x) (1 << (x))

static const char *TAG_ADS130E08 = "ads130e08";

static esp_err_t write_reg_8(ads130e08_t *dev, uint8_t reg, uint8_t val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t tx[] = { (reg | ADS130E08_CMD_WREG), 0, val, 0 };

    t.tx_buffer = tx;
    t.length = sizeof(tx) * 8;

    return spi_device_transmit(dev->spi_dev, &t);
}

static esp_err_t read_reg_8(ads130e08_t *dev, uint8_t reg, uint8_t *val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t tx[] = { (reg | ADS130E08_CMD_RREG), 0, 0, 0 };
    uint8_t rx[sizeof(tx)];

    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.length = sizeof(tx) * 8;
    CHECK(spi_device_transmit(dev->spi_dev, &t));

    *val = rx[2];

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t ads130e08_init_desc(ads130e08_t *dev, spi_host_device_t host, gpio_num_t cs_pin)
{
    CHECK_ARG(dev);

    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = cs_pin;
    dev->spi_cfg.clock_speed_hz = CLOCK_SPEED_HZ;
    dev->spi_cfg.mode = 1;
    dev->spi_cfg.queue_size = 1;
    dev->spi_cfg.cs_ena_pretrans = 1;

    return spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
}

esp_err_t ads130e08_free_desc(ads130e08_t *dev)
{
    CHECK_ARG(dev);

    return spi_bus_remove_device(dev->spi_dev);
}

esp_err_t ads130e08_send_system_cmd(ads130e08_t *dev, ads130e08_system_cmd_t cmd)
{
    CHECK_ARG(dev);

    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t tx[] = { (cmd), 0 };

    t.tx_buffer = tx;
    t.length = sizeof(tx) * 8;

    return spi_device_transmit(dev->spi_dev, &t);
}

esp_err_t ads130e08_send_data_read_cmd(ads130e08_t *dev, ads130e08_data_read_cmd_t cmd)
{
    CHECK_ARG(dev);

    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t tx[] = { (cmd), 0 };

    t.tx_buffer = tx;
    t.length = sizeof(tx) * 8;

    return spi_device_transmit(dev->spi_dev, &t);
}

esp_err_t ads130e08_get_device_id(ads130e08_t *dev, uint8_t *id)
{
    CHECK_ARG(dev);

    uint8_t val;

    CHECK(read_reg_8(dev, ADS130E08_REG_ID, &val));

    *id = val;

    return ESP_OK;
}

esp_err_t ads130e08_set_device_config(ads130e08_t *dev, ads130e08_dev_config_t config)
{
    CHECK_ARG(dev);

    uint8_t config1 = 0x00, config2 = 0x00, config3 = 0x00;

    config1 = (config.clk_en | CONFIG1_MASK_HIGH_BITS) & ~(CONFIG1_MASK_LOW_BITS);

    CHECK(write_reg_8(dev, ADS130E08_REG_CONFIG1, config1));

    config2
        = (config.int_test | config.test_amp | config.test_freq | CONFIG2_MASK_HIGH_BITS) & ~(CONFIG2_MASK_LOW_BITS);

    CHECK(write_reg_8(dev, ADS130E08_REG_CONFIG2, config2));

    config3 = (config.pd_refbuf | config.vref_4v | config.opamp_ref | config.pd_opamp | CONFIG3_MASK_HIGH_BITS)
              & ~(CONFIG3_MASK_LOW_BITS);

    CHECK(write_reg_8(dev, ADS130E08_REG_CONFIG3, config3));

    return ESP_OK;
}

esp_err_t ads130e08_get_device_config(ads130e08_t *dev, ads130e08_dev_config_t *config)
{
    CHECK_ARG(dev && config);

    uint8_t config1, config2, config3;

    CHECK(read_reg_8(dev, ADS130E08_REG_CONFIG1, &config1));

    config->clk_en = (config1 & CONFIG1_MASK_BITS_CLK_EN);

    CHECK(read_reg_8(dev, ADS130E08_REG_CONFIG2, &config2));

    config->int_test = (config2 & CONFIG2_MASK_BITS_INT_TEST);
    config->test_amp = (config2 & CONFIG2_MASK_BITS_TEST_AMP);
    config->test_freq = (config2 & CONFIG2_MASK_BITS_TEST_FREQ);

    CHECK(read_reg_8(dev, ADS130E08_REG_CONFIG3, &config3));

    config->pd_refbuf = (config3 & CONFIG3_MASK_BITS_PD_REFBUF);
    config->vref_4v = (config3 & CONFIG3_MASK_BITS_VREF_4V);
    config->opamp_ref = (config3 & CONFIG3_MASK_BITS_OPAMP_REF);
    config->pd_opamp = (config3 & CONFIG3_MASK_BITS_PD_OPAMP);

    return ESP_OK;
}

#define UPPER_1_BITS 0x80
#define LOWER_7_BITS 0x7f

esp_err_t ads130e08_get_rdata(ads130e08_t *dev, ads130e08_raw_data_t *raw_data)
{
    // (24 status bits + 16 bits × 8 channels) = 152 bits data per device + 1 dummy bit when using daisy
    // The format of the 24 status bits is (1100 + FAULT_STATP + FAULT_STATN + bits[7:4] of the GPIO: General- Purpose
    // IO Register). The data format for each channel data is twos complement, MSB first. When channels are powered down
    // using user register settings, the corresponding channel output is set to '0'.

    CHECK_ARG(dev);

    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t tx[] = { (ADS130E08_CMD_RDATA), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t rx[sizeof(tx)] = { 0 };

    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.length = sizeof(tx) * 8;

    CHECK(spi_device_transmit(dev->spi_dev, &t));

    uint32_t status_bits;

    status_bits = ((rx[1] | rx[2] | rx[3]) & 0x00FFFFFF);

    raw_data->fault_statp = (uint8_t)(status_bits & 0x000FF000);
    raw_data->fault_statn = (uint8_t)(status_bits & 0x00000FF0);
    raw_data->gpios_level = (uint8_t)(status_bits & 0x0000000F);

    uint16_t adc_raw_two_complememt;

    size_t j = 0;

    for (size_t i = 0; i < 8; i++)
    {
        adc_raw_two_complememt = ((uint16_t)(rx[4 + j] << 8) | (uint16_t)(rx[4 + j + 1] << 0));
        raw_data->channels_raw[i] = (int16_t)adc_raw_two_complememt;

        j += 2;
    }

    return ESP_OK;
}

esp_err_t ads130e08_convert_raw_to_voltage(int16_t raw, uint8_t gain, float *volts)
{
    *volts = (ADS130E08_CONVERSION_CONST * raw) / gain;

    return ESP_OK;
}

esp_err_t ads130e08_set_fault_detect_control(ads130e08_t *dev, ads130e08_fault_threshold_t fault_mode)
{
    CHECK_ARG(dev);

    uint8_t val = 0x00;

    val = (fault_mode) & ~(FAULT_MASK_LOW_BITS);

    CHECK(write_reg_8(dev, ADS130E08_REG_FAULT, val));

    return ESP_OK;
}

esp_err_t ads130e08_get_fault_detect_control(ads130e08_t *dev, ads130e08_fault_threshold_t *fault_mode)
{
    CHECK_ARG(dev);

    uint8_t val;

    CHECK(read_reg_8(dev, ADS130E08_REG_FAULT, &val));

    *fault_mode = val;

    return ESP_OK;
}

esp_err_t ads130e08_set_channel_config(ads130e08_t *dev, ads130e08_channel_t channel, ads130e08_channel_config_t config)
{
    CHECK_ARG(dev && channel);

    uint8_t val = 0x00;

    val = (config.enable | config.pga_gain | config.mode) & ~(CHnSET_MASK_LOW_BITS);

    CHECK(write_reg_8(dev, channel, val));

    return ESP_OK;
}

esp_err_t ads130e08_get_channel_config(ads130e08_t *dev, ads130e08_channel_t channel,
    ads130e08_channel_config_t *config)
{
    CHECK_ARG(dev && config);

    uint8_t val;
    CHECK(read_reg_8(dev, channel, &val));

    config->enable = val & CHnSET_MASK_BITS_PD;
    config->pga_gain = val & CHnSET_MASK_BITS_PGA;
    config->mode = val & CHnSET_MASK_BITS_MUX;

    return ESP_OK;
}

esp_err_t ads130e08_set_gpio_pin_mode(ads130e08_t *dev, ads130e08_gpio_pin_t gpio_pin, ads130e08_gpio_mode_t gpio_mode)
{
    CHECK_ARG(dev && gpio_pin && gpio_mode);

    uint8_t val;

    CHECK(read_reg_8(dev, ADS130E08_REG_GPIO, &val));

    if (gpio_pin == ADS130E08_GPIO1)
    {
        val |= (gpio_mode & MASK_BITS_GPIOC1);
    }
    else if (gpio_pin == ADS130E08_GPIO2)
    {
        val |= (gpio_mode & MASK_BITS_GPIOC2);
    }
    else if (gpio_pin == ADS130E08_GPIO3)
    {
        val |= (gpio_mode & MASK_BITS_GPIOC3);
    }
    else if (gpio_pin == ADS130E08_GPIO4)
    {
        val |= (gpio_mode & MASK_BITS_GPIOC4);
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    CHECK(write_reg_8(dev, ADS130E08_REG_GPIO, val));

    return ESP_OK;
}

esp_err_t ads130e08_get_gpio_pin_mode(ads130e08_t *dev, ads130e08_gpio_pin_t gpio_pin, ads130e08_gpio_mode_t *gpio_mode)
{
    CHECK_ARG(dev && gpio_pin && gpio_mode);

    uint8_t val;

    CHECK(read_reg_8(dev, ADS130E08_REG_GPIO, &val));

    if (gpio_pin == ADS130E08_GPIO1)
    {
        *gpio_mode = (val & MASK_BITS_GPIOC1);
    }
    else if (gpio_pin == ADS130E08_GPIO2)
    {
        *gpio_mode = (val & MASK_BITS_GPIOC2);
    }
    else if (gpio_pin == ADS130E08_GPIO3)
    {
        *gpio_mode = (val & MASK_BITS_GPIOC3);
    }
    else if (gpio_pin == ADS130E08_GPIO4)
    {
        *gpio_mode = (val & MASK_BITS_GPIOC4);
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t ads130e08_set_gpio_pin_level(ads130e08_t *dev, ads130e08_gpio_pin_t gpio_pin,
    ads130e08_gpio_level_t gpio_level)
{
    CHECK_ARG(dev && gpio_pin && gpio_level);

    uint8_t val;

    CHECK(read_reg_8(dev, ADS130E08_REG_GPIO, &val));

    if (gpio_pin == ADS130E08_GPIO1)
    {
        val |= (gpio_level & MASK_BITS_GPIOD1);
    }
    else if (gpio_pin == ADS130E08_GPIO2)
    {
        val |= (gpio_level & MASK_BITS_GPIOD2);
    }
    else if (gpio_pin == ADS130E08_GPIO3)
    {
        val |= (gpio_level & MASK_BITS_GPIOD3);
    }
    else if (gpio_pin == ADS130E08_GPIO4)
    {
        val |= (gpio_level & MASK_BITS_GPIOD4);
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    CHECK(write_reg_8(dev, ADS130E08_REG_GPIO, val));

    return ESP_OK;
}

esp_err_t ads130e08_get_gpio_pin_level(ads130e08_t *dev, ads130e08_gpio_pin_t gpio_pin,
    ads130e08_gpio_level_t *gpio_level)
{
    CHECK_ARG(dev && gpio_pin && gpio_level);

    uint8_t val;

    CHECK(read_reg_8(dev, ADS130E08_REG_GPIO, &val));

    if (gpio_pin == ADS130E08_GPIO1)
    {
        *gpio_level = (val & MASK_BITS_GPIOD1);
    }
    else if (gpio_pin == ADS130E08_GPIO2)
    {
        *gpio_level = (val & MASK_BITS_GPIOD2);
    }
    else if (gpio_pin == ADS130E08_GPIO3)
    {
        *gpio_level = (val & MASK_BITS_GPIOD3);
    }
    else if (gpio_pin == ADS130E08_GPIO4)
    {
        *gpio_level = (val & MASK_BITS_GPIOD4);
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t ads130e08_detect_fault_auto(ads130e08_t *dev, uint8_t *fault_statp, uint8_t *fault_statn)
{
    CHECK_ARG(dev);

    uint8_t fault_bits;
    CHECK(read_reg_8(dev, ADS130E08_REG_FAULT_STATP, &fault_bits));
    *fault_statp = fault_bits;

    if (fault_bits & MASK_BITS_IN1P_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on positive input 1");
    }

    if (fault_bits & MASK_BITS_IN2P_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on positive input 2");
    }

    if (fault_bits & MASK_BITS_IN3P_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on positive input 3");
    }

    if (fault_bits & MASK_BITS_IN4P_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on positive input 4");
    }

    if (fault_bits & MASK_BITS_IN5P_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on positive input 5");
    }

    if (fault_bits & MASK_BITS_IN6P_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on positive input 6");
    }

    if (fault_bits & MASK_BITS_IN7P_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on positive input 7");
    }

    if (fault_bits & MASK_BITS_IN8P_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on positive input 8");
    }

    CHECK(read_reg_8(dev, ADS130E08_REG_FAULT_STATN, &fault_bits));
    *fault_statn = fault_bits;

    if (fault_bits & MASK_BITS_IN1N_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on negative input 1");
    }

    if (fault_bits & MASK_BITS_IN2N_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on negative input 2");
    }

    if (fault_bits & MASK_BITS_IN3N_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on negative input 3");
    }

    if (fault_bits & MASK_BITS_IN4N_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on negative input 4");
    }

    if (fault_bits & MASK_BITS_IN5N_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on negative input 5");
    }

    if (fault_bits & MASK_BITS_IN6N_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on negative input 6");
    }

    if (fault_bits & MASK_BITS_IN7N_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on negative input 7");
    }

    if (fault_bits & MASK_BITS_IN8N_FAULT)
    {
        ESP_LOGE(TAG_ADS130E08, "Automatic fault detection on negative input 8");
    }

    return ESP_OK;
}
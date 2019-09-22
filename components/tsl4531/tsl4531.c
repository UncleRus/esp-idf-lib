/**
 * @file tsl4531.c
 *
 * ESP-IDF driver for I2C 16 bit GPIO expander MCP23017
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2017 Brian Schwind <https://github.com/bschwind>\n
 * Copyright (C) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include "tsl4531.h"

#define I2C_FREQ_HZ 400000

static const char *TAG = "TSL4531";

// Registers
#define TSL4531_REG_COMMAND   0x80
#define TSL4531_REG_CONTROL   0x00
#define TSL4531_REG_CONFIG    0x01
#define TSL4531_REG_DATA_LOW  0x04
#define TSL4531_REG_DATA_HIGH 0x05
#define TSL4531_REG_DEVICE_ID 0x0A

// TSL4531 Misc Values
#define TSL4531_ON  0x03
#define TSL4531_OFF 0x00

// Integration times in milliseconds
#define TSL4531_INTEGRATION_TIME_100MS 120
#define TSL4531_INTEGRATION_TIME_200MS 240
#define TSL4531_INTEGRATION_TIME_400MS 480 // Default

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t write_register(i2c_dev_t *dev, uint8_t reg, uint8_t value)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, TSL4531_REG_COMMAND | reg, &value, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t read_register(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, TSL4531_REG_COMMAND | reg, val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t read_register_16(i2c_dev_t *dev, uint8_t low_register_addr, uint16_t *val)
{
    CHECK_ARG(dev && val);

    low_register_addr = TSL4531_REG_COMMAND | low_register_addr;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, &low_register_addr, 1, val, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static inline esp_err_t enable(tsl4531_t *device)
{
    return write_register(&device->i2c_dev, TSL4531_REG_CONTROL, TSL4531_ON);
}

static inline esp_err_t disable(tsl4531_t *device)
{
    return write_register(&device->i2c_dev, TSL4531_REG_CONTROL, TSL4531_OFF);
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t tsl4531_init_desc(tsl4531_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = TSL4531_I2C_ADDR;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t tsl4531_free_desc(tsl4531_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t tsl4531_init(tsl4531_t *device)
{
    CHECK(enable(device));

    uint8_t control_reg;
    CHECK(read_register(&device->i2c_dev, TSL4531_REG_CONTROL, &control_reg));
    if (control_reg != TSL4531_ON)
    {
        ESP_LOGE(TAG, "Error initializing tsl4531, control register wasn't set to ON");
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t id;
    CHECK(read_register(&device->i2c_dev, TSL4531_REG_DEVICE_ID, &id));
    id >>= 4;
    switch (id)
    {
        case TSL4531_PART_TSL45317:
        case TSL4531_PART_TSL45313:
        case TSL4531_PART_TSL45315:
        case TSL4531_PART_TSL45311:
            device->part_id = id;
            break;
        default:
            ESP_LOGW(TAG, "Unknown part id for TSL4531 sensor: %u", id);
    }

    return disable(device);
}

esp_err_t tsl4531_config(tsl4531_t *device, tsl4531_integration_time_t integration_time, bool skip_power_save)
{
    CHECK(enable(device));
    CHECK(write_register(&device->i2c_dev, TSL4531_REG_CONFIG, (skip_power_save ? 0x08 : 0x00) | (0x03 & integration_time)));
    CHECK(disable(device));

    device->integration_time = integration_time;
    device->skip_power_save = skip_power_save;

    return ESP_OK;
}

esp_err_t tsl4531_read_lux(tsl4531_t *device, uint16_t *lux)
{
    CHECK_ARG(lux);

    CHECK(enable(device));

    uint16_t multiplier;
    switch (device->integration_time)
    {
        case TSL4531_INTEGRATION_100MS:
            multiplier = 4;
            vTaskDelay(TSL4531_INTEGRATION_TIME_100MS / portTICK_PERIOD_MS);
            break;
        case TSL4531_INTEGRATION_200MS:
            multiplier = 2;
            vTaskDelay(TSL4531_INTEGRATION_TIME_200MS / portTICK_PERIOD_MS);
            break;
        default:
            multiplier = 1;
            vTaskDelay(TSL4531_INTEGRATION_TIME_400MS / portTICK_PERIOD_MS);
    }

    uint16_t lux_data;

    CHECK(read_register_16(&device->i2c_dev, TSL4531_REG_DATA_LOW, &lux_data));
    CHECK(disable(device));

    *lux = multiplier * lux_data;

    return ESP_OK;
}

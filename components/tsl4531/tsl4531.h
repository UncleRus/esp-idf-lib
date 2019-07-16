/**
 * @file tsl4531.h
 *
 * ESP-IDF driver for I2C 16 bit GPIO expander MCP23017
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2017 Brian Schwind (https://github.com/bschwind)
 * Copyright (C) 2019 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __TSL4531_H__
#define __TSL4531_H__

#include <stdint.h>
#include <stdbool.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSL4531_I2C_ADDR 0x29

/**
 * Integration time
 */
typedef enum
{
    TSL4531_INTEGRATION_400MS = 0x00, //!< Default
    TSL4531_INTEGRATION_200MS = 0x01,
    TSL4531_INTEGRATION_100MS = 0x02,
} tsl4531_integration_time_t;

/**
 * Part IDs
 */
typedef enum
{
    TSL4531_PART_TSL45317 = 0x08,
    TSL4531_PART_TSL45313 = 0x09,
    TSL4531_PART_TSL45315 = 0x0A,
    TSL4531_PART_TSL45311 = 0x0B
} tsl4531_part_id_t;

/**
 * Device descriptor
 */
typedef struct {
    i2c_dev_t i2c_dev;
    tsl4531_integration_time_t integration_time;
    bool skip_power_save;
    tsl4531_part_id_t part_id;
} tsl4531_t;

esp_err_t tsl4531_init_desc(tsl4531_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

esp_err_t tsl4531_free_desc(tsl4531_t *dev);

esp_err_t tsl4531_init(tsl4531_t *device);

esp_err_t tsl4531_config(tsl4531_t *device, tsl4531_integration_time_t integration_time, bool skip_power_save);

esp_err_t tsl4531_read_lux(tsl4531_t *device, uint16_t *lux);

#ifdef __cplusplus
}
#endif

#endif  // __TSL4531_H__

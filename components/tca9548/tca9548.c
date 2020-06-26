/**
 * @file tca9548.c
 *
 * ESP-IDF driver for low-voltage 8-channel I2C switch TCA9548/PCA9548
 *
 * Copyright (C) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include "tca9548.h"

#define I2C_FREQ_HZ 100000 // 100kHz

static const char *TAG = "TCA9548";

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & BV(7) ? '1' : '0'), \
  (byte & BV(6) ? '1' : '0'), \
  (byte & BV(5) ? '1' : '0'), \
  (byte & BV(4) ? '1' : '0'), \
  (byte & BV(3) ? '1' : '0'), \
  (byte & BV(2) ? '1' : '0'), \
  (byte & BV(1) ? '1' : '0'), \
  (byte & BV(0) ? '1' : '0')

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t tca9548_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev && addr >= TCA9548_ADDR_0 && addr <= TCA9548_ADDR_7);

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t tca9548_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t tca9548_set_channels(i2c_dev_t *dev, uint8_t channels)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &channels, 1));
    I2C_DEV_GIVE_MUTEX(dev);
    ESP_LOGD(TAG, "[0x%02x at %d] Channels set to 0x%02x (0b" BYTE_TO_BINARY_PATTERN ")",
            dev->addr, dev->port, channels, BYTE_TO_BINARY(channels));

    return ESP_OK;
}

esp_err_t tca9548_get_channels(i2c_dev_t *dev, uint8_t *channels)
{
    CHECK_ARG(dev && channels);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, channels, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

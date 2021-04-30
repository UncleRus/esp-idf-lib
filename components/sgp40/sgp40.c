/**
 * @file sgp40.c
 *
 * ESP-IDF driver for SGP40 Indoor Air Quality Sensor for VOC Measurements
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include "sgp40.h"
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_FREQ_HZ 400000

static const char *TAG = "sgp40";

#define CMD_SOFT_RESET  0x0600 // 00 06
#define CMD_FEATURESET  0x2f20 // 20 2f
#define CMD_MEASURE_RAW 0x0f26 // 26 0f
#define CMD_SELF_TEST   0x0e28 // 28 0e
#define CMD_HEATER_OFF  0x1536 // 36 15
#define CMD_SERIAL      0x8236 // 36 82

#define TIME_SOFT_RESET  10
#define TIME_FEATURESET  10
#define TIME_MEASURE_RAW 30
#define TIME_SELF_TEST   250
#define TIME_HEATER_OFF  10
#define TIME_SERIAL      10

#define SELF_TEST_OK 0xd400

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static uint8_t calc_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xff;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static esp_err_t exec_cmd(sgp40_t *dev, const uint16_t cmd, uint8_t *params,
        size_t params_size, uint16_t *result, size_t res_len, size_t timeout)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // write
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, &cmd, 2, params, params_size));
    vTaskDelay(pdMS_TO_TICKS(timeout));
    if (!result || !res_len)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_OK;
    }
    // read
    size_t in_bytes = res_len * 3;
    uint8_t *buf = malloc(in_bytes);
    if (!buf)
    {
        ESP_LOGE(TAG, "Not enough memory");
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_ERR_NO_MEM;
    }
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, buf, in_bytes));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // check CRC & decode
    for (size_t i = 0; i < res_len; i++)
    {
        uint8_t *word = buf + i * 3;
        uint8_t crc = calc_crc(word, 2);
        if (crc != word[2])
        {
            ESP_LOGE(TAG, "CRC error: 0x%02x != 0x%02x", crc, word[2]);
            free(buf);
            return ESP_ERR_INVALID_CRC;
        }
        result[i] = (((uint16_t)word[0]) << 8) | word[1];
    }

    free(buf);
    return ESP_OK;
}

esp_err_t sgp40_init_desc(sgp40_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = SGP40_ADDR;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t sgp40_free_desc(sgp40_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sgp40_init(sgp40_t *dev)
{
    CHECK_ARG(dev);

    CHECK(exec_cmd(dev, CMD_SERIAL, NULL, 0, dev->serial, 3, TIME_SERIAL));
    CHECK(exec_cmd(dev, CMD_FEATURESET, NULL, 0, &dev->featureset, 1, TIME_FEATURESET));

    ESP_LOGI(TAG, "Device found. S/N: %04X %04X %04X, featureset 0x%04x",
            dev->serial[0], dev->serial[1], dev->serial[2], dev->featureset);

    VocAlgorithm_init(&dev->voc);

    return ESP_OK;
}

esp_err_t sgp40_soft_reset(sgp40_t *dev)
{
    CHECK_ARG(dev);

    return exec_cmd(dev, CMD_SOFT_RESET, NULL, 0, NULL, 0, TIME_SOFT_RESET);
}

esp_err_t sgp40_self_test(sgp40_t *dev)
{
    CHECK_ARG(dev);

    uint16_t res;
    CHECK(exec_cmd(dev, CMD_SELF_TEST, NULL, 0, &res, 1, TIME_SELF_TEST));

    return res == SELF_TEST_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t sgp40_heater_off(sgp40_t *dev)
{
    CHECK_ARG(dev);

    return exec_cmd(dev, CMD_HEATER_OFF, NULL, 0, NULL, 0, TIME_HEATER_OFF);
}

esp_err_t sgp40_measure_raw(sgp40_t *dev, float humidity, float temperature, uint16_t *raw)
{
    CHECK_ARG(dev && raw);

    uint8_t params[6];
    uint8_t *p = params;
    if (isnan(humidity) || isnan(temperature))
    {
        ESP_LOGW(TAG, "Uncompensated measurement");
        *p++ = 0x80; *p++ = 0x00; *p++ = 0xa2; *p++ = 0x66; *p++ = 0x66; *p++ = 0x93;
    }
    else
    {
        uint16_t h = (uint16_t)((humidity * 65535) / 100 + 0.5);
        uint16_t t = (uint16_t)(((temperature + 45) * 65535) / 175);
        *p++ = h >> 8; *p++ = h & 0xff; *p++ = calc_crc(params, 2);
        *p++ = t >> 8; *p++ = t & 0xff; *p++ = calc_crc(params + 3, 2);
    }

    return exec_cmd(dev, CMD_MEASURE_RAW, params, 6, raw, 1, TIME_MEASURE_RAW);
}

esp_err_t sgp40_measure_voc(sgp40_t *dev, float humidity, float temperature, int32_t *voc_index)
{
    CHECK_ARG(dev && voc_index);

    uint16_t raw;
    CHECK(sgp40_measure_raw(dev, humidity, temperature, &raw));
    VocAlgorithm_process(&dev->voc, raw, voc_index);

    return ESP_OK;
}

/*
 * Copyright (c) 2022 saasaa <mail@saasaa.xyz>
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

#include "htu21d.h"

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2cdev.h>

#define I2C_FREQ_HZ 400000 // 400kHz

#define TRIGGER_TEMP_MEASURE_HOLD_CMD   0xE3
#define TRIGGER_HUMD_MEASURE_HOLD_CMD   0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD_CMD 0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD_CMD 0xF5
#define WRIITE_USER_REG_CMD             0xE6
#define READ_USER_REG_CMD               0xE7
#define SOFT_RESET_CMD                  0xFE

#define USER_OTP_REL_MASK          0x02
#define USER_REG_HEAT_MASK         0x04
#define USER_REG_RESERVED_BIT_MASK 0x38
#define USER_REG_BAT_MASK          0x40
#define USER_REG_RES_MASK          0x81

#define BIT_USER_REG_HEATER 2
#define BIT_USER_REG_BAT    6

#define RESET_TIME_MS            15
#define MEASURE_TIME_RH_12B_MS   16
#define MEASURE_TIME_RH_11B_MS   8
#define MEASURE_TIME_RH_10B_MS   5
#define MEASURE_TIME_RH_8B_MS    3
#define MEASURE_TIME_TEMP_14B_MS 50
#define MEASURE_TIME_TEMP_13B_MS 25
#define MEASURE_TIME_TEMP_12B_MS 13
#define MEASURE_TIME_TEMP_11B_MS 7

#define TEMP_COEFF_MUL 175.72
#define TEMP_COEFF_ADD -46.85

#define RH_COEFF_MUL 125
#define RH_COEFF_ADD -6

#define RH_COEFF_COMP -0.15

#define CHECK(x)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        esp_err_t __;                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                        \
            return __;                                                                                                 \
    } while (0)

#define CHECK_ARG(VAL)                                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(VAL))                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                \
    } while (0)

static const char *TAG = "htu21d";

static esp_err_t check_crc(uint16_t value, uint8_t crc)
{
    uint32_t row = (uint32_t)value << 8;
    row |= crc;

    uint32_t divisor = 0x988000;

    for (int i = 0; i < 16; i++)
    {
        if (row & (uint32_t)1 << (23 - i))
            row ^= divisor;
        divisor >>= 1;
    }

    return !row ? ESP_OK : ESP_ERR_INVALID_CRC;
}

/*
 * The functions send_command_nolock and send_command are taken from the BH1750 driver and are licensed under the
 * three clause BSD license. Copyright (c) 2017 Andrej Krutak <dev@andree.sk>, 2018 Ruslan V. Uss
 * <unclerus@gmail.com> See the file components/bh1750/LICENSE for details.
 */
inline static esp_err_t send_command_nolock(htu21d_t *dev, uint8_t cmd)
{
    return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1);
}

static esp_err_t send_command(htu21d_t *dev, uint8_t cmd)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_command_nolock(dev, cmd));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_user_reg(htu21d_t *dev, uint8_t *val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, READ_USER_REG_CMD, val, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t update_user_reg(htu21d_t *dev, uint8_t val)
{
    uint8_t b;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, READ_USER_REG_CMD, &b, 1));
    b = (b & USER_REG_RESERVED_BIT_MASK) | val; // reserved bits must not be changed
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, WRIITE_USER_REG_CMD, &b, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t htu21d_init_desc(htu21d_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr != HTU21D_I2C_ADDRESS)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
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

esp_err_t htu21d_free_desc(htu21d_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t htu21d_init(htu21d_t *dev)
{
    CHECK_ARG(dev);
    ESP_ERROR_CHECK(htu21d_soft_reset(dev));

    return ESP_OK;
}

esp_err_t htu21d_soft_reset(htu21d_t *dev)
{
    CHECK_ARG(dev);

    ESP_ERROR_CHECK(send_command(dev, SOFT_RESET_CMD));
    vTaskDelay(pdMS_TO_TICKS(RESET_TIME_MS));

    return ESP_OK;
}

esp_err_t htu21d_read_data(htu21d_t *dev, float *t, float *rh)
{
    CHECK_ARG(dev && (t || rh));

    float rh_act;
    ESP_ERROR_CHECK(htu21d_read_temperature_hold(dev, t));
    ESP_ERROR_CHECK(htu21d_read_humidity_hold(dev, &rh_act));
    // ESP_ERROR_CHECK(htu21d_read_temperature_no_hold(dev, t));
    // ESP_ERROR_CHECK(htu21d_read_humidity_no_hold(dev, &rh_act));

    // Calculate compensated humidity
    *rh = rh_act + (25 - *t) * RH_COEFF_COMP;

    return ESP_OK;
}

esp_err_t htu21d_read_temperature_hold(htu21d_t *dev, float *t)
{
    CHECK_ARG(dev && t);

    uint8_t data[3];
    int16_t raw_t;
    uint16_t ms;

    switch (dev->res)
    {
        case HTU21D_RES_T_14B_RH_12B: ms = MEASURE_TIME_TEMP_14B_MS; break;
        case HTU21D_RES_T_12B_RH_8B: ms = MEASURE_TIME_TEMP_12B_MS; break;
        case HTU21D_RES_T_13B_RH_10B: ms = MEASURE_TIME_TEMP_13B_MS; break;
        case HTU21D_RES_T_11B_RH_11B: ms = MEASURE_TIME_TEMP_11B_MS; break;
        default: ms = 50; break;
    }

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_command_nolock(dev, TRIGGER_TEMP_MEASURE_HOLD_CMD));
    vTaskDelay(pdMS_TO_TICKS(ms));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, data, 3));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    raw_t = (data[0] << 8) | data[1];

    esp_err_t res = check_crc(raw_t, data[2]);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "CRC check failed");
        return res;
    }
    *t = (float)raw_t * TEMP_COEFF_MUL / (float)(1UL << 16) + TEMP_COEFF_ADD;

    return ESP_OK;
}

esp_err_t htu21d_read_humidity_hold(htu21d_t *dev, float *rh)
{
    CHECK_ARG(dev && rh);

    uint8_t data[3];
    int16_t raw_rh;
    uint16_t ms;

    switch (dev->res)
    {
        case HTU21D_RES_T_14B_RH_12B: ms = MEASURE_TIME_RH_12B_MS; break;
        case HTU21D_RES_T_12B_RH_8B: ms = MEASURE_TIME_RH_8B_MS; break;
        case HTU21D_RES_T_13B_RH_10B: ms = MEASURE_TIME_RH_10B_MS; break;
        case HTU21D_RES_T_11B_RH_11B: ms = MEASURE_TIME_RH_11B_MS; break;
        default: ms = 16; break;
    }

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_command_nolock(dev, TRIGGER_HUMD_MEASURE_HOLD_CMD));
    vTaskDelay(pdMS_TO_TICKS(ms));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, data, 3));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    raw_rh = (data[0] << 8) | data[1];

    esp_err_t res = check_crc(raw_rh, data[2]);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "CRC check failed");
        return res;
    }

    *rh = (float)raw_rh * RH_COEFF_MUL / (1UL << 16) + RH_COEFF_ADD;

    return ESP_OK;
}

esp_err_t htu21d_read_temperature_no_hold(htu21d_t *dev, float *t)
{
    CHECK_ARG(dev && t);

    uint8_t data[3];
    int16_t raw_t;
    uint16_t ms;

    switch (dev->res)
    {
        case HTU21D_RES_T_14B_RH_12B: ms = MEASURE_TIME_TEMP_14B_MS; break;
        case HTU21D_RES_T_12B_RH_8B: ms = MEASURE_TIME_TEMP_12B_MS; break;
        case HTU21D_RES_T_13B_RH_10B: ms = MEASURE_TIME_TEMP_13B_MS; break;
        case HTU21D_RES_T_11B_RH_11B: ms = MEASURE_TIME_TEMP_11B_MS; break;
        default: ms = 50; break;
    }

    I2C_DEV_CHECK(&dev->i2c_dev, send_command(dev, TRIGGER_TEMP_MEASURE_NOHOLD_CMD));
    // Waiting for measurment. Adding 5ms of safety margin.
    vTaskDelay(pdMS_TO_TICKS(ms + 5));
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, data, 3));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    raw_t = (data[0] << 8) | data[1];

    esp_err_t res = check_crc(raw_t, data[2]);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "CRC check failed");
        return res;
    }

    *t = (float)raw_t * TEMP_COEFF_MUL / (1UL << 16) + TEMP_COEFF_ADD;

    return ESP_OK;
}

esp_err_t htu21d_read_humidity_no_hold(htu21d_t *dev, float *rh)
{
    CHECK_ARG(dev && rh);

    uint8_t data[3];
    int16_t raw_rh;
    uint16_t ms;

    switch (dev->res)
    {
        case HTU21D_RES_T_14B_RH_12B: ms = MEASURE_TIME_RH_12B_MS; break;
        case HTU21D_RES_T_12B_RH_8B: ms = MEASURE_TIME_RH_8B_MS; break;
        case HTU21D_RES_T_13B_RH_10B: ms = MEASURE_TIME_RH_10B_MS; break;
        case HTU21D_RES_T_11B_RH_11B: ms = MEASURE_TIME_RH_11B_MS; break;
        default: ms = 16; break;
    }

    I2C_DEV_CHECK(&dev->i2c_dev, send_command(dev, TRIGGER_HUMD_MEASURE_NOHOLD_CMD));
    // Waiting for measurment. Adding 5ms of safety margin.
    vTaskDelay(pdMS_TO_TICKS(ms + 5));
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, data, 3));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    raw_rh = (data[0] << 8) | data[1];

    esp_err_t res = check_crc(raw_rh, data[2]);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "CRC check failed");
        return res;
    }

    *rh = (float)raw_rh * RH_COEFF_MUL / (1UL << 16) + RH_COEFF_ADD;

    return ESP_OK;
}

esp_err_t htu21d_get_resolution(htu21d_t *dev)
{
    CHECK_ARG(dev);

    uint8_t user_reg;
    ESP_ERROR_CHECK(read_user_reg(dev, &user_reg));
    dev->res = (htu21d_resolution_t)(user_reg & USER_REG_RES_MASK);

    return ESP_OK;
}

esp_err_t htu21d_set_resolution(htu21d_t *dev, htu21d_resolution_t res)
{
    CHECK_ARG(dev && res);

    ESP_ERROR_CHECK(update_user_reg(dev, (uint8_t)res));

    return ESP_OK;
}

esp_err_t htu21d_read_battery_status(htu21d_t *dev, htu21d_battery_status_t *battery_status)
{
    CHECK_ARG(dev && battery_status);

    uint8_t raw;
    ESP_ERROR_CHECK(read_user_reg(dev, &raw));

    *battery_status = (htu21d_battery_status_t)((raw & USER_REG_BAT_MASK) >> BIT_USER_REG_BAT);

    return ESP_OK;
}

esp_err_t htu21d_get_heater(htu21d_t *dev, bool *enable)
{
    CHECK_ARG(dev && enable);

    uint8_t raw;
    ESP_ERROR_CHECK(read_user_reg(dev, &raw));
    *enable = (raw & BIT(BIT_USER_REG_HEATER)) ? true : false;

    return ESP_OK;
}

esp_err_t htu21d_set_heater(htu21d_t *dev, bool enable)
{
    CHECK_ARG(dev);

    return update_user_reg(dev, enable ? BIT(BIT_USER_REG_HEATER) : 0);
}
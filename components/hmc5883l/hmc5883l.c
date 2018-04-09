/*
 * Driver for 3-axis digital compass HMC5883L
 *
 * Part of esp-open-rtos
 * Copyright (C) 2016 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#include "hmc5883l.h"

#include <sys/time.h>
#include <esp_log.h>
#include <esp_err.h>

#define REG_CR_A 0x00
#define REG_CR_B 0x01
#define REG_MODE 0x02
#define REG_DX_H 0x03
#define REG_DX_L 0x04
#define REG_DZ_H 0x05
#define REG_DZ_L 0x06
#define REG_DY_H 0x07
#define REG_DY_L 0x08
#define REG_STAT 0x09
#define REG_ID_A 0x0a
#define REG_ID_B 0x0b
#define REG_ID_C 0x0c

#define BIT_MA  5
#define BIT_DO  2
#define BIT_GN  5

#define MASK_MD 0x03
#define MASK_MA 0x60
#define MASK_DO 0x1c
#define MASK_MS 0x03
#define MASK_DR 0x01
#define MASK_DL 0x02

#define I2C_FREQ_HZ 400000

static const char *TAG = "hmc5883l";

static const float gain_values [] = {
    [HMC5883L_GAIN_1370] = 0.73,
    [HMC5883L_GAIN_1090] = 0.92,
    [HMC5883L_GAIN_820]  = 1.22,
    [HMC5883L_GAIN_660]  = 1.52,
    [HMC5883L_GAIN_440]  = 2.27,
    [HMC5883L_GAIN_390]  = 2.56,
    [HMC5883L_GAIN_330]  = 3.03,
    [HMC5883L_GAIN_230]  = 4.35
};

static float current_gain;
static hmc5883l_opmode_t current_mode;

static inline uint32_t get_time_us()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec;
}

#define timeout_expired(start, len) ((uint32_t)(get_time_us() - (start)) >= (len))
#define CHECK_VAL(VAL) do { if (!VAL) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t hmc5883l_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_VAL(dev);

    dev->port = port;
    dev->addr = HMC5883L_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
    i2c_dev_create_mutex(dev);

    return ESP_OK;
}

esp_err_t hmc5883l_free_desc(i2c_dev_t *dev)
{
    CHECK_VAL(dev);

    return i2c_dev_delete_mutex(dev);
}

static esp_err_t write_register(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    esp_err_t ret = i2c_dev_write_reg(dev, reg, &val, 1);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not write 0x%02x to register 0x%02x, err = %d", val, reg, ret);
    return ret;
}

static inline esp_err_t read_register(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    esp_err_t ret = i2c_dev_read_reg(dev, reg, val, 1);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Could not read register 0x%02x, err = %d", reg, ret);
    return ret;
}

static esp_err_t update_register(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t old;
    esp_err_t ret = read_register(dev, reg, &old);
    if (ret != ESP_OK)
        return ret;
    return write_register(dev, reg, (old & mask) | val);
}

#define CHECK(X) do { \
        esp_err_t __ = X; \
        if (__ != ESP_OK) return __; \
    } while (0)

esp_err_t hmc5883l_init(i2c_dev_t *dev)
{
    CHECK_VAL(dev);

    I2C_DEV_TAKE_MUTEX(dev);

    uint32_t id = 0;
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, REG_ID_A, &id, 3));
    if (id != HMC5883L_ID)
    {
        I2C_DEV_GIVE_MUTEX(dev);
        ESP_LOGE(TAG, "Unknown ID: 0x%08x != 0x%08x", id, HMC5883L_ID);
        return ESP_ERR_NOT_FOUND;
    }

    I2C_DEV_GIVE_MUTEX(dev);

    hmc5883l_gain_t gain;
    CHECK(hmc5883l_get_gain(dev, &gain));
    current_gain = gain_values[gain];

    CHECK(hmc5883l_get_opmode(dev, &current_mode));

    return ESP_OK;
}

esp_err_t hmc5883l_get_opmode(i2c_dev_t *dev, hmc5883l_opmode_t *val)
{
    CHECK_VAL(dev);
    CHECK_VAL(val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_register(dev, REG_MODE, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(dev);

    *val = (*val & MASK_MD) == 0 ? HMC5883L_MODE_CONTINUOUS : HMC5883L_MODE_SINGLE;
    return ESP_OK;
}

esp_err_t hmc5883l_set_opmode(i2c_dev_t *dev, hmc5883l_opmode_t mode)
{
    CHECK_VAL(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_register(dev, REG_MODE, mode));
    I2C_DEV_GIVE_MUTEX(dev);

    current_mode = mode;
    return ESP_OK;
}

esp_err_t hmc5883l_get_samples_averaged(i2c_dev_t *dev, hmc5883l_samples_averaged_t *val)
{
    CHECK_VAL(dev);
    CHECK_VAL(val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_register(dev, REG_CR_A, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(dev);

    *val = (*val & MASK_MA) >> BIT_MA;
    return ESP_OK;
}

esp_err_t hmc5883l_set_samples_averaged(i2c_dev_t *dev, hmc5883l_samples_averaged_t samples)
{
    CHECK_VAL(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_register(dev, REG_CR_A, MASK_MA, samples << BIT_MA));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t hmc5883l_get_data_rate(i2c_dev_t *dev, hmc5883l_data_rate_t *val)
{
    CHECK_VAL(val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_register(dev, REG_CR_A, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(dev);

    *val = (*val & MASK_DO) >> BIT_DO;
    return ESP_OK;
}

esp_err_t hmc5883l_set_data_rate(i2c_dev_t *dev, hmc5883l_data_rate_t rate)
{
    CHECK_VAL(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_register(dev, REG_CR_A, MASK_DO, rate << BIT_DO));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t hmc5883l_get_bias(i2c_dev_t *dev, hmc5883l_bias_t *val)
{
    CHECK_VAL(dev);
    CHECK_VAL(val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_register(dev, REG_CR_A, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(dev);

    *val &= MASK_MS;
    return ESP_OK;
}

esp_err_t hmc5883l_set_bias(i2c_dev_t *dev, hmc5883l_bias_t bias)
{
    CHECK_VAL(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_register(dev, REG_CR_A, MASK_MS, bias));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t hmc5883l_get_gain(i2c_dev_t *dev, hmc5883l_gain_t *val)
{
    CHECK_VAL(dev);
    CHECK_VAL(val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_register(dev, REG_CR_B, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(dev);

    *val >>= BIT_GN;
    return ESP_OK;
}

esp_err_t hmc5883l_set_gain(i2c_dev_t *dev, hmc5883l_gain_t gain)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_register(dev, REG_CR_B, gain << BIT_GN));
    I2C_DEV_GIVE_MUTEX(dev);

    current_gain = gain_values[gain];
    return ESP_OK;
}

esp_err_t hmc5883l_data_is_locked(i2c_dev_t *dev, bool *val)
{
    CHECK_VAL(dev);
    CHECK_VAL(val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_register(dev, REG_STAT, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(dev);

    *val &= MASK_DL;
    return ESP_OK;
}

esp_err_t hmc5883l_data_is_ready(i2c_dev_t *dev, bool *val)
{
    CHECK_VAL(dev);
    CHECK_VAL(val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_register(dev, REG_STAT, (uint8_t *)val));
    I2C_DEV_GIVE_MUTEX(dev);

    *val &= MASK_DR;
    return ESP_OK;
}

esp_err_t hmc5883l_get_raw_data(i2c_dev_t *dev, hmc5883l_raw_data_t *data)
{
    CHECK_VAL(dev);
    CHECK_VAL(data);

    if (current_mode == HMC5883L_MODE_SINGLE)
    {
        // overwrite mode register for measurement
        CHECK(hmc5883l_set_opmode(dev, current_mode));
        // wait for data
        uint32_t start = get_time_us();
        bool dready = false;
        do
        {
            CHECK(hmc5883l_data_is_ready(dev, &dready));
            if (timeout_expired(start, CONFIG_HMC5883L_MEAS_TIMEOUT))
                return ESP_ERR_TIMEOUT;
        } while (!dready);
    }
    uint8_t buf[6];
    uint8_t reg = REG_DX_H;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, buf, 6));
    I2C_DEV_GIVE_MUTEX(dev);

    data->x = ((int16_t)buf[REG_DX_H - REG_DX_H] << 8) | buf[REG_DX_L - REG_DX_H];
    data->y = ((int16_t)buf[REG_DY_H - REG_DX_H] << 8) | buf[REG_DY_L - REG_DX_H];
    data->z = ((int16_t)buf[REG_DZ_H - REG_DX_H] << 8) | buf[REG_DZ_L - REG_DX_H];

    return ESP_OK;
}

void hmc5883l_raw_to_mg(const hmc5883l_raw_data_t *raw, hmc5883l_data_t *mg)
{
    mg->x = raw->x * current_gain;
    mg->y = raw->y * current_gain;
    mg->z = raw->z * current_gain;
}

esp_err_t hmc5883l_get_data(i2c_dev_t *dev, hmc5883l_data_t *data)
{
    hmc5883l_raw_data_t raw;

    CHECK(hmc5883l_get_raw_data(dev, &raw));

    hmc5883l_raw_to_mg(&raw, data);
    return ESP_OK;
}

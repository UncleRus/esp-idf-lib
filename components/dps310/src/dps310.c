/*
 * Copyright (c) 2022 Tomoyuki Sakurai <y@trombik.org>
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

/**
 * @file dps310.c
 *
 * ESP-IDF driver for DPS310. Sponserd by @beriberikix.
 *
 * Note that the driver always tries to compensate raw values from the sensor.
 * When compensation is not required, or undesired, users should implement
 * their own functions to bypass compensation.
 */

/* standard headers */
#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include <math.h>

/* esp-idf headers */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <esp_err.h>
#include <i2cdev.h>

/* private headers */
#include "helper_macro.h"
#include "helper_i2c.h"

/* public headers */
#include "dps310.h"

#define DPS310_QUIRK_DELAY_MS       (10)
#define DPS310_QUIRK_MAX_ATTEMPT    (5)

static const char *TAG = "dps310";

/* see 4.9.3 Compensation Scale Factors */
#define N_SCALE_FACTORS (8)
static const int32_t scale_factors[N_SCALE_FACTORS] = {
    524288,
    1572864,
    3670016,
    7864320,
    253952,
    516096,
    1040384,
    2088960
};

esp_err_t dps310_quirk(dps310_t *dev)
{
    esp_err_t err = ESP_FAIL;
    const int magic_command_len = 5;
    float ignore = 0;
    const uint8_t magic_commands[5][2] = {

        /* reg address, value */
        { 0xA5, 0x0E },
        { 0x0F, 0x96 },
        { 0x62, 0x02 },
        { 0x0E, 0x00 },
        { 0x0F, 0x00 },
    };
    dps310_mode_t original_mode = 0;

    CHECK_ARG(dev);
    ESP_LOGD(TAG, "dps310_quirk(): resetting resisters with magic numbers");
    for (int i = 0; i < magic_command_len; i++)
    {
        uint8_t reg = magic_commands[i][0];
        uint8_t value = magic_commands[i][1];
        err = _write_reg(&dev->i2c_dev, reg, &value);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "_write_reg(): %s (reg: 0x%02X)", esp_err_to_name(err), reg);
            break;
        }
    }
    if (err != ESP_OK)
    {
        goto fail;
    }
    vTaskDelay(pdMS_TO_TICKS(DPS310_STARTUP_DELAY_MS));

    /* sensor is ready? */
    err = _wait_for_reg_bits(&dev->i2c_dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_SENSOR_RDY_MASK, 1, DPS310_QUIRK_MAX_ATTEMPT, DPS310_QUIRK_DELAY_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "_wait_for_reg_bits(): %s", esp_err_to_name(err));
        goto fail;
    }
    /* coef is ready? */
    err = _wait_for_reg_bits(&dev->i2c_dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_COEF_RDY_MASK, 1, DPS310_QUIRK_MAX_ATTEMPT, DPS310_QUIRK_DELAY_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "_wait_for_reg_bits(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGD(TAG, "dps310_quirk(): reading COEF");
    err = dps310_get_coef(dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_get_coef(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGD(TAG, "dps310_quirk(): keep the original operation mode");
    err = dps310_get_mode(dev, &original_mode);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_get_mode(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGD(TAG, "dps310_quirk(): setting mode to DPS310_MODE_COMMAND_TEMPERATURE");
    err = dps310_set_mode(dev, DPS310_MODE_COMMAND_TEMPERATURE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dps310_set_mode(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGD(TAG, "dps310_quirk(): reading temperature just once");
    err = dps310_read_temp_wait(dev, DPS310_QUIRK_DELAY_MS, DPS310_QUIRK_MAX_ATTEMPT, &ignore);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_temp_wait(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGD(TAG, "dps310_quirk(): restore the original mode");
    err = dps310_set_mode(dev, original_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dps310_set_mode(): %s", esp_err_to_name(err));
        goto fail;
    }

fail:
    return err;
}

esp_err_t dps310_init_desc(dps310_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    esp_err_t err = ESP_FAIL;

    if (dev == NULL)
    {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    if (addr != DPS310_I2C_ADDRESS_0 && addr != DPS310_I2C_ADDRESS_1)
    {
        ESP_LOGE(TAG, "Invalid I2C address: expected %0X or %0X, got %0X",
                 DPS310_I2C_ADDRESS_0, DPS310_I2C_ADDRESS_1, addr);
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = DPS310_I2C_FREQ_MAX_ESP_IDF_HZ;
#endif

    ESP_LOGD(TAG, "Port: %u SCL: GPIO%i SDA: GPIO%i",
             port,
             dev->i2c_dev.cfg.scl_io_num,
             dev->i2c_dev.cfg.sda_io_num);
    err = i2c_dev_create_mutex(&dev->i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_create_mutex(): %s", esp_err_to_name(err));
        goto fail;
    }
    dev->pressure_s = DPS310_AVERAGE_SEA_LEVEL_PRESSURE_Pa;
    dev->offset = 0;

fail:
    return err;

}

esp_err_t dps310_free_desc(dps310_t *dev)
{
    esp_err_t err = ESP_FAIL;

    err = i2c_dev_delete_mutex(&dev->i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "i2c_dev_delete_mutex(): %s", esp_err_to_name(err));
        goto fail;
    }
fail:
    return err;
}

static int pow_int(int base, int x)
{
    int result = 1;

    for (int i = 0; i < x ; ++i)
    {
        result *= base;
    }
    return result;
}

esp_err_t dps310_init(dps310_t *dev, dps310_config_t *config)
{
    uint8_t reg_value = 0;
    esp_err_t err = ESP_FAIL;

    if (dev == NULL)
    {
        ESP_LOGE(TAG, "dps310_init(): invalid dev");
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    if (config == NULL)
    {
        ESP_LOGE(TAG, "dps310_init(): invalid config");
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    err = _read_reg(&dev->i2c_dev, DPS310_REG_ID, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Device not found");
        goto fail;
    }

    dev->prod_id = (DPS310_REG_ID_PROD_MASK & reg_value) >> 4;
    dev->prod_rev = DPS310_REG_ID_REV_MASK & reg_value;
    ESP_LOGD(TAG, "prod_id: 0x%x prod_rev: 0x%x", dev->prod_id, dev->prod_rev);
    if (dev->prod_id != DPS310_PROD_ID)
    {
        ESP_LOGE(TAG, "Invalid prod ID: expected: 0x%x (DPS310) got: 0x%x", DPS310_PROD_ID, dev->prod_id);
        err = ESP_FAIL;
        goto fail;
    }

    err = dps310_reset(dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_reset(): %s", esp_err_to_name(err));
        goto fail;
    }

    err = dps310_quirk(dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_quirk(): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGD(TAG, "Pressure measurement rate: %i measurements / sec", pow_int(2, config->pm_rate));
    err = dps310_set_rate_p(dev, config->pm_rate);
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGD(TAG, "Pressure oversampling: %i time(s)", pow_int(2, config->pm_oversampling));
    err = dps310_set_oversampling_p(dev, config->pm_oversampling);
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGD(TAG, "Temperature measurement rate: %i measurements / sec", pow_int(2, config->tmp_rate));
    err = dps310_set_rate_t(dev, config->tmp_rate);
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGD(TAG, "Temperature oversampling: %i time(s)", pow_int(2, config->tmp_oversampling));
    err = dps310_set_oversampling_t(dev, config->tmp_oversampling);
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGD(TAG, "Temperature source: %s", config->tmp_src == DPS310_TMP_SRC_INTERNAL ? "internal" : "external");
    err = dps310_set_tmp_ext(dev, config->tmp_src);
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGD(TAG, "Temperature COEF source: %s", config->tmp_coef == DPS310_TMP_SRC_INTERNAL ? "internal" : "external");
    err = dps310_set_tmp_coef_ext(dev, config->tmp_coef);
    if (err != ESP_OK)
    {
        goto fail;
    }
    if (config->tmp_src != config->tmp_coef)
    {

        /* XXX don't know when DPS310_TMP_SRC_INTERNAL should be used. the
         * datasheet does not mention the differece.
         */
        ESP_LOGW(TAG, "tmp_src and tmp_coef should be an identical source. Use DPS310_TMP_SRC_EXTERNAL in the config");
    }
    ESP_LOGD(TAG, "Interrupt FIFO: %s", config->int_fifo_mode == DPS310_INT_FIFO_ENABLE ? "enabled" : "disabled");
    err = dps310_set_int_fifo(dev, config->int_fifo_mode);
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGD(TAG, "Interrupt temperature: %s", config->int_tmp_mode == DPS310_INT_TMP_ENABLE ? "enabled" : "disabled");
    err = dps310_set_int_tmp(dev, config->int_tmp_mode);
    if (err != ESP_OK)
    {
        goto fail;
    }
    ESP_LOGD(TAG, "Interrupt pressure: %s", config->int_prs_mode == DPS310_INT_PRS_ENABLE ? "enabled" : "disabled");
    err = dps310_set_int_prs(dev, config->int_prs_mode);
    if (err != ESP_OK)
    {
        goto fail;
    }

    ESP_LOGD(TAG, "Temperature result bit-shift: %s", config->t_shift_mode == DPS310_T_SHIFT_ENABLE ? "enabled" : "disabled");
    if (config->tmp_oversampling > DPS310_TMP_PRC_8 && config->t_shift_mode != DPS310_T_SHIFT_ENABLE)
    {
        ESP_LOGW(TAG, "Temperature result bit-shift must be enabled, but is disabled. Set DPS310_T_SHIFT_ENABLE");
    }
    err = dps310_set_t_shift(dev, config->t_shift_mode);
    if (err != ESP_OK)
    {
        goto fail;
    }

    ESP_LOGD(TAG, "Pressure result bit-shift: %s", config->p_shift_mode == DPS310_P_SHIFT_ENABLE ? "enabled" : "disabled");
    if (config->pm_oversampling > DPS310_PM_PRC_8 && config->p_shift_mode != DPS310_P_SHIFT_ENABLE)
    {
        ESP_LOGW(TAG, "Pressure result bit-shift must be enabled, but is disabled. Set DPS310_P_SHIFT_ENABLE");
    }
    err = dps310_set_p_shift(dev, config->p_shift_mode);
    if (err != ESP_OK)
    {
        goto fail;
    }

    ESP_LOGD(TAG, "FIFO: %s", config->fifo_en_mode == DPS310_FIFO_ENABLE ? "enabled" : "disabled");
    err = dps310_set_fifo_en(dev, config->fifo_en_mode);
    if (err != ESP_OK)
    {
        goto fail;
    }

    err = ESP_OK;

fail:
    return err;
}

esp_err_t dps310_get_rate_p(dps310_t *dev, dps310_pm_rate_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_PRS_CFG, DPS310_REG_PRS_CFG_PM_RATE_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_rate_p(dps310_t *dev, dps310_pm_rate_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_PRS_CFG, DPS310_REG_PRS_CFG_PM_RATE_MASK, value);
}

esp_err_t dps310_get_rate_t(dps310_t *dev, dps310_tmp_rate_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_TMP_CFG, DPS310_REG_PRS_CFG_TMP_RATE_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_rate_t(dps310_t *dev, dps310_tmp_rate_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_TMP_CFG, DPS310_REG_PRS_CFG_TMP_RATE_MASK, value);
}

esp_err_t dps310_reset(dps310_t *dev)
{
    esp_err_t err = ESP_FAIL;
    uint8_t value = DPS310_SOFT_RST_VALUE;

    CHECK_ARG(dev);

    ESP_LOGD(TAG, "Resetting the device");
    err = _write_reg(&dev->i2c_dev, DPS310_REG_RESET, &value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to reset the device");
        goto fail;
    }
    vTaskDelay(pdMS_TO_TICKS(DPS310_STARTUP_DELAY_MS));
fail:
    return err;
}

esp_err_t dps310_get_oversampling_p(dps310_t *dev, dps310_pm_oversampling_t *value)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev && value);
    err = _read_reg_mask(&dev->i2c_dev, DPS310_REG_PRS_CFG, DPS310_REG_PRS_CFG_PM_PRC_MASK, (uint8_t *)value);
    if (err == ESP_OK)
    {
        /* XXX when new p_rate is available, always keep it in dev as a cache */
        dev->p_rate = *value;
    }
    return err;
}

esp_err_t dps310_set_oversampling_p(dps310_t *dev, dps310_pm_oversampling_t value)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev);
    err = _update_reg(&dev->i2c_dev, DPS310_REG_PRS_CFG, DPS310_REG_PRS_CFG_PM_PRC_MASK, value);
    if (err == ESP_OK)
    {
        /* XXX when new p_rate is available, always keep it in dev as a cache */
        dev->p_rate = value;
    }
    return err;
}

esp_err_t dps310_get_oversampling_t(dps310_t *dev, dps310_tmp_oversampling_t *value)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev && value);
    err = _read_reg_mask(&dev->i2c_dev, DPS310_REG_TMP_CFG, DPS310_REG_TMP_CFG_TMP_PRC_MASK, (uint8_t *)value);
    if (err == ESP_OK)
    {
        /* XXX when new t_rate is available, always keep it in dev as a cache */
        dev->t_rate = *value;
    }
    return err;
}

esp_err_t dps310_set_oversampling_t(dps310_t *dev, dps310_tmp_oversampling_t value)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev);
    err = _update_reg(&dev->i2c_dev, DPS310_REG_TMP_CFG, DPS310_REG_TMP_CFG_TMP_PRC_MASK, value);
    if (err == ESP_OK)
    {
        /* XXX when new t_rate is available, always keep it in dev as a cache */
        dev->t_rate = value;
    }
    return err;
}

esp_err_t dps310_get_tmp_ext(dps310_t *dev, dps310_tmp_src_ext_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_TMP_CFG, DPS310_REG_PRS_CFG_TMP_EXT_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_tmp_ext(dps310_t *dev, dps310_tmp_src_ext_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_TMP_CFG, DPS310_REG_PRS_CFG_TMP_EXT_MASK, value);
}

esp_err_t dps310_set_tmp_coef_ext(dps310_t *dev, dps310_tmp_src_ext_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_COEF_SRCE, DPS310_REG_COEF_SRCE_MASK, value);
}

esp_err_t dps310_get_int_hl(dps310_t *dev, dps310_int_hl_active_level_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_INT_HL_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_int_hl(dps310_t *dev, dps310_int_hl_active_level_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_PRS_CFG_TMP_EXT_MASK, value);
}

esp_err_t dps310_get_int_fifo(dps310_t *dev, dps310_int_fifo_mode_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_INT_FIFO_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_int_fifo(dps310_t *dev, dps310_int_fifo_mode_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_INT_FIFO_MASK, value);
}

esp_err_t dps310_get_int_tmp(dps310_t *dev, dps310_int_tmp_mode_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_INT_TMP_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_int_tmp(dps310_t *dev, dps310_int_tmp_mode_t value)
{
    return _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_INT_TMP_MASK, value);
}

esp_err_t dps310_get_int_prs(dps310_t *dev, dps310_int_prs_mode_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_INT_PRS_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_int_prs(dps310_t *dev, dps310_int_prs_mode_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_INT_PRS_MASK, value);
}

esp_err_t dps310_get_t_shift(dps310_t *dev, dps310_t_shift_mode_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_T_SHIFT_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_t_shift(dps310_t *dev, dps310_t_shift_mode_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_T_SHIFT_MASK, value);
}

esp_err_t dps310_get_p_shift(dps310_t *dev, dps310_p_shift_mode_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_P_SHIFT_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_p_shift(dps310_t *dev, dps310_p_shift_mode_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_P_SHIFT_MASK, value);
}

esp_err_t dps310_get_fifo_en(dps310_t *dev, dps310_fifo_en_mode_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_FIFO_EN_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_fifo_en(dps310_t *dev, dps310_fifo_en_mode_t value)
{
    return _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_FIFO_EN_MASK, value);
}

esp_err_t dps310_get_spi_mode(dps310_t *dev, dps310_spi_mode_t *value)
{
    CHECK_ARG(dev && value);

    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_SPI_MODE_MASK, (uint8_t *)value);
}

esp_err_t dps310_set_spi_mode(dps310_t *dev, dps310_spi_mode_t value)
{
    CHECK_ARG(dev);

    return _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_SPI_MODE_MASK, value);
}

static int32_t two_complement_of(uint32_t value, uint8_t length)
{
    int32_t result = (uint32_t)value;
    if (value & ((uint32_t)1 << (length - (uint32_t)1)))
    {
        result -= ((uint32_t) 1 << length);
    }
    return result;
}

esp_err_t dps310_get_coef(dps310_t *dev)
{
    uint8_t reg_values[DPS310_REG_COEF_LEN];

    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    err = i2c_dev_read_reg(&dev->i2c_dev, DPS310_REG_COEF, reg_values, DPS310_REG_COEF_LEN);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_read_reg(): %s", esp_err_to_name(err));
        goto fail;
    }

    dev->coef.c0 = ((uint32_t)reg_values[0] << 4) | (((uint32_t)reg_values[1] >> 4) & 0x0F);
    dev->coef.c0 = two_complement_of(dev->coef.c0, 12);

    dev->coef.c1 = (((uint32_t)reg_values[1] & 0x0F) << 8) | (uint32_t)reg_values[2];
    dev->coef.c1 = two_complement_of(dev->coef.c1, 12);

    dev->coef.c00 = ((uint32_t)reg_values[3] << 12) | ((uint32_t)reg_values[4] << 4) | (((uint32_t)reg_values[5] >> 4) & 0x0F);
    dev->coef.c00 = two_complement_of(dev->coef.c00, 20);

    dev->coef.c10 = (((uint32_t)reg_values[5] & 0x0F) << 16) | ((uint32_t)reg_values[6] << 8) | (uint32_t)reg_values[7];
    dev->coef.c10 = two_complement_of(dev->coef.c10, 20);

    dev->coef.c01 = ((uint32_t)reg_values[8] << 8) | (uint32_t)reg_values[9];
    dev->coef.c01 = two_complement_of(dev->coef.c01, 16);

    dev->coef.c11 = ((uint32_t)reg_values[10] << 8) | (uint32_t)reg_values[11];
    dev->coef.c11 = two_complement_of(dev->coef.c11, 16);

    dev->coef.c20 = ((uint32_t)reg_values[12] << 8) | (uint32_t)reg_values[13];
    dev->coef.c20 = two_complement_of(dev->coef.c20, 16);

    dev->coef.c21 = ((uint32_t)reg_values[14] << 8) | (uint32_t)reg_values[15];
    dev->coef.c21 = two_complement_of(dev->coef.c21, 16);

    dev->coef.c30 = ((uint32_t)reg_values[16] << 8) | (uint32_t)reg_values[17];
    dev->coef.c30 = two_complement_of(dev->coef.c30, 16);
fail:
    return err;

}

esp_err_t dps310_get_mode(dps310_t *dev, dps310_mode_t *mode)
{
    return _read_reg_mask(&dev->i2c_dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_MEAS_CTRL_MASK, (uint8_t *)mode);
}

esp_err_t dps310_set_mode(dps310_t *dev, dps310_mode_t mode)
{
    return _update_reg(&dev->i2c_dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_MEAS_CTRL_MASK, mode);
}

esp_err_t dps310_flush_fifo(dps310_t *dev)
{
    uint8_t value = DPS310_FIFO_FLUSH_VALUE;

    CHECK_ARG(dev);
    return _write_reg(&dev->i2c_dev, DPS310_REG_RESET, &value);
}

esp_err_t dps310_enable_fifo(dps310_t *dev, bool enable)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev);
    if (!enable)
    {
        err = dps310_flush_fifo(dev);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dps310_flush_fifo(): %s", esp_err_to_name(err));
            goto fail;
        }
    }
    err = _update_reg(&dev->i2c_dev, DPS310_REG_CFG_REG, DPS310_REG_CFG_REG_FIFO_EN_MASK, enable ? 1 : 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "_update_reg(): %s", esp_err_to_name(err));
        goto fail;
    }
fail:
    return err;
}

esp_err_t dps310_read_raw(dps310_t *dev, uint8_t reg, int32_t *value)
{
    uint8_t reg_values[DPS310_REG_SENSOR_VALUE_LEN] = {0};
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev && value);
    if (reg != DPS310_REG_TMP_B2 && reg != DPS310_REG_PRS_B2)
    {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    err = i2c_dev_read_reg(&dev->i2c_dev, reg, reg_values, DPS310_REG_SENSOR_VALUE_LEN);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_read_reg(): %s", esp_err_to_name(err));
        goto fail;
    }
    *value = ((uint32_t)reg_values[0] << 16) | ((uint32_t)reg_values[1] << 8) | (uint32_t)reg_values[2];
    *value = two_complement_of(*value, DPS310_REG_SENSOR_VALUE_LEN * 8);
    ESP_LOGD(TAG, "raw value in %s resister: %" PRIi32, reg == DPS310_REG_TMP_B2 ? "temperature" : "pressure", *value);
fail:
    return err;
}

/* calcurate and return kT, or kP. */
static float raw_to_scaled(int32_t raw, uint8_t rate)
{
    int32_t k = 0;

    assert(rate <= N_SCALE_FACTORS - 1);
    k = scale_factors[rate];
    ESP_LOGD(TAG, "scale_factor: %" PRIi32, k);

    /* Traw_sc = Traw / kT */
    assert(k != 0);
    return (float)raw / (float)k;
}

static float compensate_temp(dps310_t *dev, uint32_t T_raw, uint8_t rate)
{

    /* 4.9.2 How to Calculate Compensated Temperature Values */
    float result = 0;
    float T_raw_scaled = 0;

    CHECK_ARG(dev);
    T_raw_scaled = raw_to_scaled(T_raw, rate);

    /* Tcomp (°C) = c0 * 0.5 + c1 * Traw_sc */
    result = ((float)dev->coef.c0 * 0.5) + ((float)dev->coef.c1 * T_raw_scaled);
    return result;
}

static float compensate_pressure(dps310_t *dev, int32_t T_raw, int32_t T_rate, int32_t P_raw, int32_t P_rate)
{

    /* 4.9.1 How to Calculate Compensated Pressure Values */
    float T_raw_scaled = 0;
    float P_raw_scaled = 0;

    CHECK_ARG(dev);

    T_raw_scaled = raw_to_scaled(T_raw, T_rate);
    P_raw_scaled = raw_to_scaled(P_raw, P_rate);

    /* Pcomp(Pa) = c00
     *             + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30))
     *             + Traw_sc * c01
     *             + Traw_sc * Praw_sc * (c11 + Praw_sc * c21)
     */
    return (float)dev->coef.c00
           + P_raw_scaled * ((float)dev->coef.c10 + P_raw_scaled * ((float)dev->coef.c20 + P_raw_scaled * (float)dev->coef.c30))
           + T_raw_scaled * (float)dev->coef.c01
           + T_raw_scaled * P_raw_scaled * ((float)dev->coef.c11 + P_raw_scaled * (float)dev->coef.c21);
}

esp_err_t dps310_read_pressure(dps310_t *dev, float *pressure)
{
    esp_err_t err = ESP_FAIL;
    int32_t T_raw = 0;
    int32_t P_raw = 0;
    dps310_tmp_oversampling_t T_rate = 0;
    dps310_pm_oversampling_t P_rate = 0;

    CHECK_ARG(dev && pressure);
    err = dps310_get_oversampling_t(dev, &T_rate);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_get_oversampling_t(): %s", esp_err_to_name(err));
        goto fail;
    }

    err = dps310_get_oversampling_p(dev, &P_rate);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_get_oversampling_p(): %s", esp_err_to_name(err));
        goto fail;
    }

    err = dps310_read_raw(dev, DPS310_REG_TMP_B2, &T_raw);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_raw(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_read_raw(dev, DPS310_REG_PRS_B2, &P_raw);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_raw(): %s", esp_err_to_name(err));
        goto fail;
    }
    *pressure = compensate_pressure(dev, T_raw, T_rate, P_raw, P_rate);
fail:
    return err;
}

esp_err_t dps310_read_pressure_wait(dps310_t *dev, uint16_t delay_ms, uint8_t max_attempt, float *pressure)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev && pressure);
    err = _wait_for_reg_bits(&dev->i2c_dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_PRS_RDY_MASK, 1, max_attempt, delay_ms);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "_wait_for_reg_bits(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_read_pressure(dev, pressure);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_pressure(): %s", esp_err_to_name(err));
        goto fail;
    }
fail:
    return err;
}

esp_err_t dps310_read_temp(dps310_t *dev, float *temperature)
{
    esp_err_t err = ESP_FAIL;
    int32_t T_raw = 0;
    dps310_tmp_oversampling_t rate = 0;

    CHECK_ARG(dev && temperature);
    err = dps310_get_oversampling_t(dev, &rate);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_get_oversampling_t(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_read_raw(dev, DPS310_REG_TMP_B2, &T_raw);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_raw(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* XXX when latest t_raw is available, always keep it in dev as a cache */
    dev->t_raw = T_raw;
    *temperature = compensate_temp(dev, dev->t_raw, rate);
fail:
    return err;
}

esp_err_t dps310_read_temp_wait(dps310_t *dev, uint16_t delay_ms, uint8_t max_attempt, float *temperature)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev && temperature);
    err = _wait_for_reg_bits(&dev->i2c_dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_TMP_RDY_MASK, 1, max_attempt, delay_ms);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "_wait_for_reg_bits(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_read_temp(dev, temperature);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_temp(): %s", esp_err_to_name(err));
        goto fail;
    }
fail:
    return err;
}

esp_err_t dps310_is_ready_for(dps310_t *dev, uint8_t reg, uint8_t mask, bool *ready)
{
    esp_err_t err = ESP_FAIL;
    uint8_t reg_value = 0;

    CHECK_ARG(dev && ready);

    err = _read_reg_mask(&dev->i2c_dev, reg, mask, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "_read_reg_mask(): %s", esp_err_to_name(err));
        goto fail;
    }
    *ready = reg_value == 1 ? true : false;
fail:
    return err;
}

esp_err_t dps310_is_ready_for_coef(dps310_t *dev, bool *ready)
{
    CHECK_ARG(dev && ready);
    return dps310_is_ready_for(dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_COEF_RDY_MASK, ready);
}

esp_err_t dps310_is_ready_for_sensor(dps310_t *dev, bool *ready)
{
    CHECK_ARG(dev && ready);
    return dps310_is_ready_for(dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_SENSOR_RDY_MASK, ready);
}

esp_err_t dps310_is_ready_for_temp(dps310_t *dev, bool *ready)
{
    CHECK_ARG(dev && ready);
    return dps310_is_ready_for(dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_TMP_RDY_MASK, ready);
}

esp_err_t dps310_is_ready_for_pressure(dps310_t *dev, bool *ready)
{
    CHECK_ARG(dev && ready);
    return dps310_is_ready_for(dev, DPS310_REG_MEAS_CFG, DPS310_REG_MEAS_CFG_PRS_RDY_MASK, ready);
}

static inline bool dps310_is_pressure_result(int32_t data)
{
    return (uint8_t)data & 0x01;
}

static inline bool dps310_is_temp_result(int32_t data)
{
    return !dps310_is_pressure_result(data);
}

static esp_err_t dps310_read_reg_sensor_raw(dps310_t *dev, uint8_t reg, int32_t *value)
{
    uint8_t reg_values[DPS310_REG_SENSOR_VALUE_LEN] = {0};
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev && value);
    if (reg != DPS310_REG_TMP_B2 && reg != DPS310_REG_PRS_B2)
    {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    err = i2c_dev_read_reg(&dev->i2c_dev, reg, reg_values, DPS310_REG_SENSOR_VALUE_LEN);
    *value = (uint32_t)reg_values[0] << 16 | (uint32_t)reg_values[1] << 8 | (uint32_t)reg_values[2];
    ESP_LOGD(TAG, "reg_values: %" PRIi32, *value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_read_reg(): %s", esp_err_to_name(err));
        goto fail;
    }
fail:
    return err;
}

esp_err_t dps310_is_fifo_empty(dps310_t *dev, bool *result)
{
    esp_err_t err = ESP_FAIL;
    uint8_t value = 0;

    CHECK_ARG(dev && result);
    err = _read_reg_mask(&dev->i2c_dev, DPS310_REG_FIFO_STS, DPS310_REG_FIFO_STS_FIFO_EMPTY_MASK, &value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "_read_reg_mask(): %s", esp_err_to_name(err));
        goto fail;
    }
    *result = value == 1 ? true : false;

fail:
    return err;

}

esp_err_t dps310_read_fifo(dps310_t *dev, dps310_fifo_measurement_t *measurement)
{
    int32_t raw_value = 0;
    esp_err_t err = ESP_OK;

    CHECK_ARG(dev && measurement);

    /* Read a measurement from FIFO. to compensate the value, additional
     * parameters, such as t_rate, are necessary. Use cached parameters in the
     * device descriptor so that the value can be returned by only one I2C
     * reading. This means that the driver does not work in multi-master
     * configuration. As long as cached parameters are in sync with the real
     * values in registers, the returned value is always correct even if a
     * parameter is modified during the background mode.
     */
    err = dps310_read_reg_sensor_raw(dev, DPS310_REG_FIFO, &raw_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_reg_sensor_raw(): %s", esp_err_to_name(err));
        goto fail;
    }
    measurement->type = dps310_is_pressure_result(raw_value) ? DPS310_MEASUREMENT_PRESSURE : DPS310_MEASUREMENT_TEMPERATURE;
    raw_value = two_complement_of(raw_value, 24);
    if (raw_value == DPS310_FIFO_EMPTY)
    {
        measurement->type = DPS310_MEASUREMENT_EMPTY;
    }

    switch (measurement->type)
    {
        case DPS310_MEASUREMENT_TEMPERATURE:
            ESP_LOGD(TAG, "DPS310_MEASUREMENT_TEMPERATURE: raw_value: %" PRIi32, raw_value);
            measurement->result = compensate_temp(dev, raw_value, dev->t_rate);

            /* XXX when new t_raw is available, always keep it in dev as a
             * cache */
            dev->t_raw = raw_value;
            break;
            ;;
        case DPS310_MEASUREMENT_PRESSURE:
            ESP_LOGD(TAG, "DPS310_MEASUREMENT_PRESSURE: raw_value: %" PRIi32, raw_value);
            measurement->result = compensate_pressure(dev, dev->t_raw, dev->t_rate, raw_value, dev->p_rate);
            break;
            ;;
        case DPS310_MEASUREMENT_EMPTY:
            ESP_LOGD(TAG, "DPS310_MEASUREMENT_EMPTY: raw_value %" PRIi32, raw_value);
            measurement->result = 0;
            break;
        default:

            /* NOT REACHED */
            abort();
            ;;
    }

fail:
    return err;
}

inline esp_err_t dps310_backgorund_start(dps310_t *dev, dps310_mode_t mode)
{
    CHECK_ARG(dev);
    return dps310_set_mode(dev, mode);
}

inline esp_err_t dps310_backgorund_stop(dps310_t *dev)
{
    CHECK_ARG(dev);
    return dps310_set_mode(dev, DPS310_MODE_STANDBY);
}

static int32_t dps310_calc_sea_level_pressure(float pressure, float altitude)
{
    return (int32_t)(pressure / pow(1.0 - altitude / 44330, 5.255));
}

static inline float calc_altitude(float pressure, float pressure_s)
{
    return 44330 * (1.0 - pow(pressure / pressure_s, 0.1903));
}

esp_err_t dps310_calibrate_altitude(dps310_t *dev, float altitude_real)
{
    esp_err_t err = ESP_FAIL;
    float pressure = 0;
    float temperature = 0;
    float altitude_guess = 0;
    dps310_pm_oversampling_t orig_oversmapling_p = 0;
    dps310_tmp_oversampling_t orig_oversmapling_t = 0;

    CHECK_ARG(dev);

    /* keep original oversampling values */
    err = dps310_get_oversampling_p(dev, &orig_oversmapling_p);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_get_oversampling_p(): %s", esp_err_to_name(err));
        goto get_oversampling_fail;
    }
    err = dps310_get_oversampling_t(dev, &orig_oversmapling_t);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_get_oversampling_t(): %s", esp_err_to_name(err));
        goto get_oversampling_fail;
    }

    /* modify oversampling values for accuracy */
    err = dps310_set_oversampling_p(dev, DPS310_PM_PRC_64);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_oversampling_p(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_set_oversampling_t(dev, DPS310_TMP_PRC_64);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_oversampling_t(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* read temperature once in command mode */
    err = dps310_set_mode(dev, DPS310_MODE_COMMAND_TEMPERATURE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_mode(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_read_temp_wait(dev, 100, 10, &temperature);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_temp(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* read pressure once in command mode */
    err = dps310_set_mode(dev, DPS310_MODE_COMMAND_PRESSURE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_set_mode(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_read_pressure_wait(dev, 100, 10, &pressure);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_pressure(): %s", esp_err_to_name(err));
        goto fail;
    }

    /* calibrate offset */
    dev->pressure_s = dps310_calc_sea_level_pressure(pressure, altitude_real);
    altitude_guess = calc_altitude(pressure, dev->pressure_s);
    dev->offset = altitude_real - altitude_guess;

    ESP_LOGI(TAG, "Calibration result:");
    ESP_LOGI(TAG, "\tPressure: %0.2f (Pa)", pressure);
    ESP_LOGI(TAG, "\tCalculated Pressure at sea level: %0.2f (Pa)", dev->pressure_s);
    ESP_LOGI(TAG, "\tCalculated altitude: %0.2f (m)", altitude_guess);
    ESP_LOGI(TAG, "\tReal altitude: %0.2f (m)", altitude_real);
    ESP_LOGI(TAG, "\tOffset: %0.2f (m)", dev->offset);
fail:
    if (dps310_set_oversampling_t(dev, orig_oversmapling_t) != ESP_OK)
    {
        ESP_LOGW(TAG, "Restoring temperature oversampling failed");
    }
    if (dps310_set_oversampling_p(dev, orig_oversmapling_p) != ESP_OK)
    {
        ESP_LOGW(TAG, "Restoring pressure oversampling failed");
    }
get_oversampling_fail:
    return err;
}


static float pressure_to_altitude(float pressure, float pressure_s, float *altitude)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(altitude);
    if (pressure_s == 0)
    {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    *altitude = 44330 * (1.0 - pow(pressure / pressure_s, 0.1903));
    err = ESP_OK;

fail:
    return err;
}

esp_err_t dps310_calc_altitude(dps310_t *dev, float pressure, float *altitude)
{
    esp_err_t err = ESP_FAIL;

    CHECK_ARG(dev && altitude);

    err = pressure_to_altitude(pressure, dev->pressure_s, altitude);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "pressure_to_altitude(): %s", esp_err_to_name(err));
        goto fail;
    }
    *altitude = *altitude + dev->offset;
fail:
    return err;
}

esp_err_t dps310_read_altitude(dps310_t *dev, float *altitude)
{
    esp_err_t err = ESP_FAIL;
    float pressure = 0;

    CHECK_ARG(dev && altitude);

    err = dps310_read_pressure(dev, &pressure);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_read_pressure(): %s", esp_err_to_name(err));
        goto fail;
    }
    err = dps310_calc_altitude(dev, pressure, altitude);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_calc_altitude(): %s", esp_err_to_name(err));
        goto fail;
    }

fail:
    return err;
}

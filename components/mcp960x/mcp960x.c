/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file mcp960x.c
 *
 * ESP-IDF driver for MCP960X/L0X/RL0X
 * Thermocouple EMF to Temperature Converter
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "mcp960x.h"

#define I2C_FREQ_HZ 100000 // 100 KHz

static const char *TAG = "mcp960x";

#define REG_T_HOT      0x00
#define REG_T_DELTA    0x01
#define REG_T_COLD     0x02
#define REG_RAW        0x03
#define REG_STATUS     0x04
#define REG_SENS_CONF  0x05
#define REG_DEV_CONF   0x06
#define REG_ALERT_CONF 0x08
#define REG_ALERT_HYST 0x0c
#define REG_ALERT_LIM  0x10
#define REG_DEV_ID     0x20

#define BIT_ST_ALERT1 0
#define BIT_ST_ALERT2 1
#define BIT_ST_ALERT3 2
#define BIT_ST_ALERT4 3
#define BIT_ST_OC     4
#define BIT_ST_SC     5
#define BIT_ST_TH     6
#define BIT_ST_BURST  7

#define MASK_ST_MODE    (3 << BIT_ST_OC)

#define BIT_AC_OUT_EN   0
#define BIT_AC_COMP_INT 1
#define BIT_AC_ACT_LVL  2
#define BIT_AC_T_DIR    3
#define BIT_AC_T_SRC    4
#define BIT_AC_INT_CLR  7

#define BIT_SC_FILTER  0
#define BIT_SC_TC_TYPE 4

#define MASK_SC_FILTER  7
#define MASK_SC_TC_TYPE (7 << BIT_SC_TC_TYPE)

#define BIT_DC_MODE     0
#define BIT_DC_SAMPLES  2
#define BIT_DC_ADC_RES  5
#define BIT_DC_SENS_RES 7

#define MASK_DC_MODE    (3 << BIT_DC_MODE)
#define MASK_DC_SAMPLES (7 << BIT_DC_SAMPLES)
#define MASK_DC_ADC_RES (3 << BIT_DC_ADC_RES)

#define BV(x) (1 << (x))

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define LSB 0.0625

inline static uint16_t shuffle(uint16_t x)
{
    return (x >> 8) | (x << 8);
}

inline static esp_err_t write_reg_8(mcp960x_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1);
}

inline static esp_err_t read_reg_8(mcp960x_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(&dev->i2c_dev, reg, val, 1);
}

static esp_err_t read_reg_8_lock(mcp960x_t *dev, uint8_t reg, uint8_t *val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t write_reg_8_lock(mcp960x_t *dev, uint8_t reg, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t write_reg_16(mcp960x_t *dev, uint8_t reg, uint16_t val)
{
    uint16_t v = shuffle(val);
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &v, 2);
}

static esp_err_t read_reg_16(mcp960x_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, reg, val, 2));
    *val = shuffle(*val);
    return ESP_OK;
}

inline static esp_err_t write_reg_16_float(mcp960x_t *dev, uint8_t reg, float val)
{
    return write_reg_16(dev, reg, (uint16_t)(val / LSB));
}

static esp_err_t read_reg_16_float(mcp960x_t *dev, uint8_t reg, float *val)
{
    uint16_t raw;
    CHECK(read_reg_16(dev, reg, &raw));
    *val = (float)raw * LSB;
    return ESP_OK;
}

static esp_err_t read_reg_16_float_lock(mcp960x_t *dev, uint8_t reg, float *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_16_float(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}


///////////////////////////////////////////////////////////////////////////////

esp_err_t mcp960x_init_desc(mcp960x_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr < (MCP960X_ADDR_BASE) || addr > (MCP960X_ADDR_DEFAULT))
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

    // Min. stretch time ~70us
    dev->i2c_dev.timeout_ticks = I2CDEV_MAX_STRETCH_TIME;

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t mcp960x_free_desc(mcp960x_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t mcp960x_init(mcp960x_t *dev)
{
    CHECK_ARG(dev);

    uint16_t r;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_16(dev, REG_DEV_ID, &r));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->revision = r;
    dev->id = r >> 8;

    ESP_LOGD(TAG, "Device found, ID:Rev 0x%02x:0x%02x", dev->id, dev->revision);

    return ESP_OK;
}

esp_err_t mcp960x_set_sensor_config(mcp960x_t *dev, mcp960x_thermocouple_t th, uint8_t filter)
{
    CHECK_ARG(dev && th <= MCP960X_TYPE_R && filter <= MCP960X_FILTER_MAX);

    return write_reg_8_lock(dev, REG_SENS_CONF, (th << BIT_SC_TC_TYPE) | filter);
}

esp_err_t mcp960x_get_sensor_config(mcp960x_t *dev, mcp960x_thermocouple_t *th, uint8_t *filter)
{
    CHECK_ARG(dev && (th || filter));

    uint8_t r;
    CHECK(read_reg_8_lock(dev, REG_SENS_CONF, &r));

    if (th)
        *th = (r & MASK_SC_TC_TYPE) >> BIT_SC_TC_TYPE;
    if (filter)
        *filter = (r & MASK_SC_FILTER) >> BIT_SC_FILTER;

    return ESP_OK;
}

esp_err_t mcp960x_set_device_config(mcp960x_t *dev, mcp960x_mode_t mode, mcp960x_burst_samples_t bs,
        mcp960x_adc_resolution_t adc_res, mcp960x_tc_resolution_t tc_res)
{
    CHECK_ARG(dev
            && mode <= MCP960X_MODE_BURST
            && bs <= MCP960X_SAMPLES_128
            && adc_res <= MCP960X_ADC_RES_12
            && tc_res <= MCP960X_TC_RES_0_25);

    return write_reg_8_lock(dev, REG_DEV_CONF,
            (mode << BIT_DC_MODE) |
            (bs << BIT_DC_SAMPLES) |
            (adc_res << BIT_DC_ADC_RES) |
            (tc_res << BIT_DC_SENS_RES));
}

esp_err_t mcp960x_get_device_config(mcp960x_t *dev, mcp960x_mode_t *mode, mcp960x_burst_samples_t *bs,
        mcp960x_adc_resolution_t *adc_res, mcp960x_tc_resolution_t *tc_res)
{
    CHECK_ARG(dev && (mode || bs || adc_res || tc_res));

    uint8_t r;
    CHECK(read_reg_8_lock(dev, REG_SENS_CONF, &r));

    if (mode)
        *mode = (r & MASK_DC_MODE) >> BIT_DC_MODE;
    if (bs)
        *bs = (r & MASK_DC_SAMPLES) >> BIT_DC_SAMPLES;
    if (adc_res)
        *adc_res = (r & MASK_DC_ADC_RES) >> BIT_DC_ADC_RES;
    if (tc_res)
        *tc_res = (r >> BIT_DC_SENS_RES) & 1;

    return ESP_OK;
}

esp_err_t mcp960x_set_mode(mcp960x_t *dev, mcp960x_mode_t mode)
{
    CHECK_ARG(dev && mode <= MCP960X_MODE_BURST);

    uint8_t r;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8(dev, REG_DEV_CONF, &r));
    r = (r & ~MASK_DC_MODE) | (mode << BIT_DC_MODE);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8(dev, REG_DEV_CONF, r));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mcp960x_get_raw_adc_data(mcp960x_t *dev, int32_t *data)
{
    CHECK_ARG(dev && data);

    uint8_t raw[3];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_RAW, raw, 3));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    uint32_t v = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2];
    if (v & 0x800000) v |= 0xff000000;
    *((uint32_t *)data) = v;

    return ESP_OK;
}

esp_err_t mcp960x_get_thermocouple_temp(mcp960x_t *dev, float *t)
{
    return read_reg_16_float_lock(dev, REG_T_HOT, t);
}

esp_err_t mcp960x_get_delta_temp(mcp960x_t *dev, float *t)
{
    return read_reg_16_float_lock(dev, REG_T_DELTA, t);
}

esp_err_t mcp960x_get_ambient_temp(mcp960x_t *dev, float *t)
{
    return read_reg_16_float_lock(dev, REG_T_COLD, t);
}

esp_err_t mcp960x_get_status(mcp960x_t *dev, bool *temp_ready, bool *burst_ready, mcp960x_status_t *status,
        bool *alert1, bool *alert2, bool *alert3, bool *alert4)
{
    CHECK_ARG(dev && (temp_ready || burst_ready || status || alert1 || alert2 || alert3 || alert4));

    uint8_t r;
    CHECK(read_reg_8_lock(dev, REG_STATUS, &r));

    if (temp_ready)
        *temp_ready = (r >> BIT_ST_TH) & 1;
    if (burst_ready)
        *burst_ready = (r >> BIT_ST_BURST) & 1;
    if (status)
       *status = (r & MASK_ST_MODE) >> BIT_ST_OC;
    if (alert1)
        *alert1 = (r >> BIT_ST_ALERT1) & 1;
    if (alert2)
        *alert2 = (r >> BIT_ST_ALERT2) & 1;
    if (alert3)
        *alert3 = (r >> BIT_ST_ALERT3) & 1;
    if (alert4)
        *alert4 = (r >> BIT_ST_ALERT4) & 1;

    return ESP_OK;
}

esp_err_t mcp960x_set_alert_config(mcp960x_t *dev, mcp960x_alert_t alert, mcp960x_alert_mode_t mode,
        mcp960x_alert_level_t active_lvl, mcp960x_alert_temp_dir_t temp_dir, mcp960x_alert_source_t src,
        float limit, uint8_t hyst)
{
    CHECK_ARG(dev
            && alert <= MCP960X_ALERT_4
            && mode <= MCP960X_ALERT_INT
            && active_lvl <= MCP960X_ACTIVE_HIGH
            && temp_dir <= MCP960X_RISING
            && src <= MCP960X_ALERT_SRC_TC);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8(dev, REG_ALERT_CONF + alert,
            (mode > MCP960X_ALERT_DISABLED ? BV(BIT_AC_OUT_EN) : 0) |
            (mode > MCP960X_ALERT_DISABLED ? (mode - 1) << BIT_AC_COMP_INT : 0) |
            (active_lvl << BIT_AC_ACT_LVL) |
            (temp_dir << BIT_AC_T_DIR) |
            (src << BIT_AC_T_SRC)));
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8(dev, REG_ALERT_HYST + alert, hyst));
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_16_float(dev, REG_ALERT_LIM + alert, limit));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mcp960x_get_alert_config(mcp960x_t *dev, mcp960x_alert_t alert, mcp960x_alert_mode_t *mode,
        mcp960x_alert_level_t *active_lvl, mcp960x_alert_temp_dir_t *temp_dir, mcp960x_alert_source_t *src,
        float *limit, uint8_t *hyst)
{
    CHECK_ARG(dev && alert <= MCP960X_ALERT_4
            && (mode || active_lvl || temp_dir || src || limit || hyst));

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    if (mode || active_lvl || temp_dir || src)
    {
        uint8_t v;
        I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8(dev, REG_ALERT_CONF + alert, &v));
        if (mode)
            *mode = v & BV(BIT_AC_OUT_EN) ? ((v >> BIT_AC_COMP_INT) & 1) + 1 : 0;
        if (active_lvl)
            *active_lvl = (v >> BIT_AC_ACT_LVL) & 1;
        if (temp_dir)
            *temp_dir = (v >> BIT_AC_T_DIR) & 1;
        if (src)
            *src = (v >> BIT_AC_T_SRC) & 1;
    }
    if (limit)
        I2C_DEV_CHECK(&dev->i2c_dev, read_reg_16_float(dev, REG_ALERT_LIM + alert, limit));
    if (hyst)
        I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8(dev, REG_ALERT_LIM + alert, hyst));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t mcp960x_get_alert_status(mcp960x_t *dev, mcp960x_alert_t alert, bool *status)
{
    CHECK_ARG(dev && alert <= MCP960X_ALERT_4 && status);

    uint8_t r;
    CHECK(read_reg_8_lock(dev, REG_STATUS, &r));
    *status = (r >> (BIT_ST_ALERT1 + alert)) & 1;

    return ESP_OK;
}

esp_err_t mcp960x_clear_alert_int(mcp960x_t *dev, mcp960x_alert_t alert)
{
    CHECK_ARG(dev && alert <= MCP960X_ALERT_4);

    uint8_t r;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8(dev, REG_ALERT_CONF + alert, &r));
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8(dev, REG_ALERT_CONF + alert, r | BV(BIT_AC_INT_CLR)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}


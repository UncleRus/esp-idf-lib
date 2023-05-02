/*
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file ccs811.c
 *
 * ESP-IDF driver for AMS CCS811 digital gas sensor connected to I2C
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>\n
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <string.h>
#include <inttypes.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ccs811.h"

#define I2C_FREQ_HZ 400000 // 400kHz max

static const char *TAG = "ccs811";

/* CCS811 register addresses */
#define CCS811_REG_STATUS          0x00
#define CCS811_REG_MEAS_MODE       0x01
#define CCS811_REG_ALG_RESULT_DATA 0x02
#define CCS811_REG_RAW_DATA        0x03
#define CCS811_REG_ENV_DATA        0x05
#define CCS811_REG_NTC             0x06
#define CCS811_REG_THRESHOLDS      0x10
#define CCS811_REG_BASELINE        0x11

#define CCS811_REG_HW_ID           0x20
#define CCS811_REG_HW_VER          0x21
#define CCS811_REG_FW_BOOT_VER     0x23
#define CCS811_REG_FW_APP_VER      0x24

#define CCS811_REG_ERROR_ID        0xe0

#define CCS811_REG_APP_ERASE       0xf1
#define CCS811_REG_APP_DATA        0xf2
#define CCS811_REG_APP_VERIFY      0xf3
#define CCS811_REG_APP_START       0xf4
#define CCS811_REG_SW_RESET        0xff

// status register bits
#define CCS811_STATUS_ERROR        0x01  // error, details in CCS811_REG_ERROR
#define CCS811_STATUS_DATA_RDY     0x08  // new data sample in ALG_RESULT_DATA
#define CCS811_STATUS_APP_VALID    0x10  // valid application firmware loaded
#define CCS811_STATUS_FW_MODE      0x80  // firmware is in application mode

// error register bits
#define CCS811_ERR_WRITE_REG_INV   0x01  // invalid register address on write
#define CCS811_ERR_READ_REG_INV    0x02  // invalid register address on read
#define CCS811_ERR_MEASMODE_INV    0x04  // invalid requested measurement mode
#define CCS811_ERR_MAX_RESISTANCE  0x08  // maximum sensor resistance exceeded 
#define CCS811_ERR_HEATER_FAULT    0x10  // heater current not in range
#define CCS811_ERR_HEATER_SUPPLY   0x20  // heater voltage not applied correctly

/**
 * Type declarations
 */

typedef struct
{
    uint8_t reserved_1 :2;
    uint8_t int_thresh :1;  // interrupt if new ALG_RESULT_DAT crosses on of the thresholds
    uint8_t int_datardy:1;  // interrupt if new sample is ready in ALG_RESULT_DAT
    uint8_t drive_mode :3;  // mode number binary coded
} ccs811_meas_mode_reg_t;

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

///////////////////////////////////////////////////////////////////////////////
/// Static functions

static esp_err_t read_reg_nolock(ccs811_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len)
{
    ESP_LOGD(TAG, "Read %" PRIu32 " bytes from i2c slave starting at reg addr %02x.", len, reg);

    esp_err_t res = i2c_dev_read_reg(&dev->i2c_dev, reg, data, len);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error %d on read %" PRIu32 " bytes from I2C slave reg addr %02x.", res, len, reg);
        return res;
    }

    return ESP_OK;
}

static esp_err_t write_reg_nolock(ccs811_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len)
{
    ESP_LOGD(TAG, "Write %" PRIu32 " bytes to i2c slave starting at reg addr %02x", len, reg);

    esp_err_t res = i2c_dev_write_reg(&dev->i2c_dev, reg, data, len);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error %d on write %" PRIu32 " bytes to i2c slave register %02x.", res, len, reg);
        return res;
    }

    return ESP_OK;
}

static esp_err_t ccs811_is_available(ccs811_dev_t *dev)
{
    CHECK_ARG(dev);

    uint8_t reg_data[5];

    // check hardware id (register 0x20) and hardware version (register 0x21)
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, CCS811_REG_HW_ID, reg_data, 5));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (reg_data[0] != 0x81)
    {
        ESP_LOGE(TAG, "Wrong hardware ID %02x, should be 0x81", reg_data[0]);
        return CCS811_ERR_HW_ID;
    }

    ESP_LOGD(TAG, "hardware version:      %02x", reg_data[1]);
    ESP_LOGD(TAG, "firmware boot version: %02x", reg_data[3]);
    ESP_LOGD(TAG, "firmware app version:  %02x", reg_data[4]);

    return ESP_OK;
}

static esp_err_t ccs811_enable_threshold(ccs811_dev_t *dev, bool enabled)
{
    CHECK_ARG(dev);

    ccs811_meas_mode_reg_t reg;

    // first, enable/disable the data ready interrupt
    CHECK(ccs811_enable_interrupt(dev, enabled));

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // read measurement mode register value
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, CCS811_REG_MEAS_MODE, (uint8_t *)&reg, 1));

    // second, enable/disable the threshold interrupt mode
    reg.int_thresh = enabled;

    // write back measurement mode register
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
            write_reg_nolock(dev, CCS811_REG_MEAS_MODE, (uint8_t *)&reg, 1),
            "Could not set measurement mode register.");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t ccs811_check_error_status(ccs811_dev_t *dev)
{
    uint8_t status;
    uint8_t err_reg;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // check status register
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, CCS811_REG_STATUS, &status, 1));

    if (!status & CCS811_STATUS_ERROR)
    {
        // everything is fine
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_OK;
    }

    // Check the error register
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, CCS811_REG_ERROR_ID, &err_reg, 1));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (err_reg & CCS811_ERR_WRITE_REG_INV)
    {
        ESP_LOGE(TAG, "Received an invalid register for write.");
        return CCS811_ERR_WR_REG_INV;
    }

    if (err_reg & CCS811_ERR_READ_REG_INV)
    {
        ESP_LOGE(TAG, "Received an invalid register for read.");
        return CCS811_ERR_RD_REG_INV;
    }

    if (err_reg & CCS811_ERR_MEASMODE_INV)
    {
        ESP_LOGE(TAG, "Received an invalid measurement mode request.");
        return CCS811_ERR_MM_INV;
    }

    if (err_reg & CCS811_ERR_MAX_RESISTANCE)
    {
        ESP_LOGE(TAG, "Sensor resistance measurement has reached or exceeded the maximum range.");
        return CCS811_ERR_MAX_RESIST;
    }

    if (err_reg & CCS811_ERR_HEATER_FAULT)
    {
        ESP_LOGE(TAG, "Heater current not in range.");
        return CCS811_ERR_HEAT_FAULT;
    }

    if (err_reg & CCS811_ERR_HEATER_SUPPLY)
    {
        ESP_LOGE(TAG, "Heater voltage is not being applied correctly.");
        return CCS811_ERR_HEAT_SUPPLY;
    }

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// Public functions

esp_err_t ccs811_init_desc(ccs811_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr != CCS811_I2C_ADDRESS_1 && addr != CCS811_I2C_ADDRESS_2)
    {
        ESP_LOGE(TAG, "Invalid device address: 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    dev->i2c_dev.timeout_ticks = I2CDEV_MAX_STRETCH_TIME;

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t ccs811_free_desc(ccs811_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t ccs811_init(ccs811_dev_t *dev)
{
    CHECK_ARG(dev);

    // init sensor data structure
    dev->mode = CCS811_MODE_IDLE;

    // check whether sensor is available including the check of the hardware
    // id and the error state
    CHECK_LOGE(ccs811_is_available(dev), "Sensor is not available.");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    const static uint8_t sw_reset[4] = { 0x11, 0xe5, 0x72, 0x8a };

    // doing a software reset first
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
            write_reg_nolock(dev, CCS811_REG_SW_RESET, (uint8_t *)sw_reset, 4),
            "Could not reset the sensor.");

    uint8_t status;

    // wait 100 ms after the reset
    vTaskDelay(pdMS_TO_TICKS(100));

    // get the status to check whether sensor is in bootloader mode
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
            read_reg_nolock(dev, CCS811_REG_STATUS, &status, 1),
            "Could not read status register 0x%02x.", CCS811_REG_STATUS);

    // if sensor is in bootloader mode (FW_MODE == 0), it has to switch
    // to the application mode first
    if (!(status & CCS811_STATUS_FW_MODE))
    {
        // check whether valid application firmware is loaded
        if (!(status & CCS811_STATUS_APP_VALID))
        {
            ESP_LOGE(TAG, "Sensor is in boot mode, but has no valid application.");
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
            return CCS811_ERR_NO_APP;
        }

        // swtich to application mode
        uint8_t r = CCS811_REG_APP_START;
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
                i2c_dev_write(&dev->i2c_dev, NULL, 0, &r, 1),
                "Could not start application.");

        // wait 100 ms after starting the app
        vTaskDelay(pdMS_TO_TICKS(100));

        // get the status to check whether sensor switched to application mode
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
                read_reg_nolock(dev, CCS811_REG_STATUS, &status, 1),
                "Could not read application status.");
        if (!(status & CCS811_STATUS_FW_MODE))
        {
            ESP_LOGE(TAG, "Could not start application, invalid status 0x%02x.", status);
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
            return CCS811_ERR_APP_START_FAIL;
        }
    }

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // try to set default measurement mode to CCS811_MODE_1S
    CHECK(ccs811_set_mode(dev, CCS811_MODE_1S));

    return ESP_OK;
}

esp_err_t ccs811_set_mode(ccs811_dev_t *dev, ccs811_mode_t mode)
{
    CHECK_ARG(dev);

    ccs811_meas_mode_reg_t reg;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // read measurement mode register value
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, CCS811_REG_MEAS_MODE, (uint8_t *)&reg, 1));

    reg.drive_mode = mode;

    // write back measurement mode register
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
            write_reg_nolock(dev, CCS811_REG_MEAS_MODE, (uint8_t *)&reg, 1),
            "Could not set measurement mode.");

    // check whether setting measurement mode were succesfull
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
            read_reg_nolock(dev, CCS811_REG_MEAS_MODE, (uint8_t *)&reg, 1),
            "Could not set measurement mode.");

    if (reg.drive_mode != mode)
    {
        ESP_LOGE(TAG, "Could not set measurement mode to %d", mode);
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return CCS811_ERR_MM_INV;
    }

    dev->mode = mode;
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

#define CCS811_ALG_DATA_ECO2_HB   0
#define CCS811_ALG_DATA_ECO2_LB   1
#define CCS811_ALG_DATA_TVOC_HB   2
#define CCS811_ALG_DATA_TVOC_LB   3
#define CCS811_ALG_DATA_STATUS    4
#define CCS811_ALG_DATA_ERROR_ID  5
#define CCS811_ALG_DATA_RAW_HB    6
#define CCS811_ALG_DATA_RAW_LB    7

esp_err_t ccs811_get_results(ccs811_dev_t *dev, uint16_t *iaq_tvoc,
        uint16_t *iaq_eco2, uint8_t *raw_i, uint16_t *raw_v)
{
    CHECK_ARG(dev);

    if (dev->mode == CCS811_MODE_IDLE)
    {
        ESP_LOGE(TAG, "Sensor is in idle mode and not performing measurements.");
        return CCS811_ERR_WRONG_MODE;
    }

    if (dev->mode == CCS811_MODE_250MS && (iaq_tvoc || iaq_eco2))
    {
        ESP_LOGE(TAG, "Sensor is in constant power mode, only raw data are available every 250ms");
        return CCS811_ERR_NO_IAQ_DATA;
    }

    uint8_t data[8];

    // read IAQ sensor values and RAW sensor data including status and error id
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
            read_reg_nolock(dev, CCS811_REG_ALG_RESULT_DATA, data, 8),
            "Could not read sensor data.");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // check for errors
    if (data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_ERROR)
        return ccs811_check_error_status(dev);

    // check whether new data are ready, if not, latest values are read from sensor
    if (!(data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_DATA_RDY))
    {
        ESP_LOGD(TAG, "No new data.");
        return CCS811_ERR_NO_NEW_DATA;
    }

    // if *iaq* is not NULL return IAQ sensor values
    if (iaq_tvoc)
        *iaq_tvoc = data[CCS811_ALG_DATA_TVOC_HB] << 8 | data[CCS811_ALG_DATA_TVOC_LB];
    if (iaq_eco2)
        *iaq_eco2 = data[CCS811_ALG_DATA_ECO2_HB] << 8 | data[CCS811_ALG_DATA_ECO2_LB];

    // if *raw* is not NULL return RAW sensor data
    if (raw_i)
        *raw_i = data[CCS811_ALG_DATA_RAW_HB] >> 2;
    if (raw_v)
        *raw_v = (data[CCS811_ALG_DATA_RAW_HB] & 0x03) << 8 | data[CCS811_ALG_DATA_RAW_LB];

    return ESP_OK;
}

esp_err_t ccs811_get_ntc_resistance(ccs811_dev_t *dev, uint32_t r_ref,
        uint32_t *res)
{
    CHECK_ARG(dev && res);

    uint8_t data[4];

    // read baseline register
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, CCS811_REG_NTC, data, 4));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // calculation from application note ams AN000372
    uint16_t v_ref = (uint16_t) (data[0]) << 8 | data[1];
    uint16_t v_ntc = (uint16_t) (data[2]) << 8 | data[3];

    *res = (v_ntc * r_ref / v_ref);

    return ESP_OK;
}

esp_err_t ccs811_set_environmental_data(ccs811_dev_t *dev,
        float temperature, float humidity)
{
    CHECK_ARG(dev);

    uint16_t hum_conv = humidity * 512.0f + 0.5f;
    uint16_t temp_conv = (temperature + 25.0f) * 512.0f + 0.5f;
    

    // fill environmental data
    uint8_t data[4] = {
        (uint8_t)((hum_conv >> 8) & 0xFF), (uint8_t)(hum_conv & 0xFF),
        (uint8_t)((temp_conv >> 8) & 0xFF), (uint8_t)(temp_conv & 0xFF)
    };

    // send environmental data to the sensor
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
            write_reg_nolock(dev, CCS811_REG_ENV_DATA, data, 4),
            "Could not write environmental data to sensor.");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t ccs811_set_eco2_thresholds(ccs811_dev_t *dev, uint16_t low,
        uint16_t high, uint8_t hysteresis)
{
    CHECK_ARG(dev);

    // check whether interrupt has to be disabled
    if (!low && !high && !hysteresis)
        return ccs811_enable_threshold(dev, false);

    // check parameters
    if (low < CCS_ECO2_RANGE_MIN || high > CCS_ECO2_RANGE_MAX || low > high || !hysteresis)
    {
        ESP_LOGE(TAG, "Wrong threshold parameters");
        CHECK(ccs811_enable_threshold(dev, false));
        return CCS811_ERR_WRONG_PARAMS;
    }

    // fill the threshold data
    uint8_t data[5] = { low >> 8, low & 0xff, high >> 8, high & 0xff, hysteresis };

    // write threshold data to the sensor
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev,
            write_reg_nolock(dev, CCS811_REG_THRESHOLDS, data, 5),
            "Could not write threshold interrupt data to sensor.");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // finally enable the threshold interrupt mode    
    return ccs811_enable_threshold(dev, true);
}

esp_err_t ccs811_enable_interrupt(ccs811_dev_t *dev, bool enabled)
{
    CHECK_ARG(dev);

    ccs811_meas_mode_reg_t reg;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // read measurement mode register value
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, CCS811_REG_MEAS_MODE, (uint8_t *)&reg, 1));

    reg.int_datardy = enabled;
    reg.int_thresh = false;      // threshold mode must not enabled

    // write back measurement mode register
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, CCS811_REG_MEAS_MODE, (uint8_t *)&reg, 1));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t ccs811_get_baseline(ccs811_dev_t *dev, uint16_t *baseline)
{
    CHECK_ARG(dev && baseline);

    uint8_t data[2];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    // read baseline register
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, CCS811_REG_BASELINE, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *baseline = (uint16_t) (data[0]) << 8 | data[1];

    return ESP_OK;
}

esp_err_t ccs811_set_baseline(ccs811_dev_t *dev, uint16_t baseline)
{
    CHECK_ARG(dev);

    uint8_t data[2] = { baseline >> 8, baseline & 0xff };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    // write baseline register
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, CCS811_REG_BASELINE, data, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

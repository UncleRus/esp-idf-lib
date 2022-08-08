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
 * ESP-IDF driver for BME680 digital environmental sensor
 *
 * Forked from <https://github.com/gschorcht/bme680-esp-idf>
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include "bme680.h"

#define I2C_FREQ_HZ 1000000 // Up to 3.4MHz, but esp-idf only supports 1MHz

// modes: unfortunatly, only SLEEP_MODE and FORCED_MODE are documented
#define BME680_SLEEP_MODE           0x00    // low power sleeping
#define BME680_FORCED_MODE          0x01    // perform one TPHG cycle (field data 0 filled)
#define BME680_PARALLEL_MODE        0x02    // no information what it does :-(
#define BME680_SQUENTUAL_MODE       0x02    // no information what it does (field data 0+1+2 filled)

// register addresses
#define BME680_REG_RES_HEAT_VAL     0x00
#define BME680_REG_RES_HEAT_RANGE   0x02
#define BME680_REG_RANGE_SW_ERROR   0x06

#define BME680_REG_IDAC_HEAT_BASE   0x50    // 10 regsrs idac_heat_0 ... idac_heat_9
#define BME680_REG_RES_HEAT_BASE    0x5a    // 10 registers res_heat_0 ... res_heat_9
#define BME680_REG_GAS_WAIT_BASE    0x64    // 10 registers gas_wait_0 ... gas_wait_9
#define BME680_REG_CTRL_GAS_0       0x70
#define BME680_REG_CTRL_GAS_1       0x71
#define BME680_REG_CTRL_HUM         0x72
#define BME680_REG_STATUS           0x73
#define BME680_REG_CTRL_MEAS        0x74
#define BME680_REG_CONFIG           0x75
#define BME680_REG_ID               0xd0
#define BME680_REG_RESET            0xe0

// field data 0 registers
#define BME680_REG_MEAS_STATUS_0    0x1d
#define BME680_REG_MEAS_INDEX_0     0x1e
#define BME680_REG_PRESS_MSB_0      0x1f
#define BME680_REG_PRESS_LSB_0      0x20
#define BME680_REG_PRESS_XLSB_0     0x21
#define BME680_REG_TEMP_MSB_0       0x22
#define BME680_REG_TEMP_LSB_0       0x23
#define BME680_REG_TEMP_XLSB_0      0x24
#define BME680_REG_HUM_MSB_0        0x25
#define BME680_REG_HUM_LSB_0        0x26
#define BME680_REG_GAS_R_MSB_0      0x2a
#define BME680_REG_GAS_R_LSB_0      0x2b

// field data 1 registers (not documented, used in SEQUENTIAL_MODE)
#define BME680_REG_MEAS_STATUS_1    0x2e
#define BME680_REG_MEAS_INDEX_1     0x2f

// field data 2 registers (not documented, used in SEQUENTIAL_MODE)
#define BME680_REG_MEAS_STATUS_2    0x3f
#define BME680_REG_MEAS_INDEX_2     0x40

// field data addresses
#define BME680_REG_RAW_DATA_0       BME680_REG_MEAS_STATUS_0    // 0x1d ... 0x2b
#define BME680_REG_RAW_DATA_1       BME680_REG_MEAS_STATUS_1    // 0x2e ... 0x3c
#define BME680_REG_RAW_DATA_2       BME680_REG_MEAS_STATUS_2    // 0x40 ... 0x4d
#define BME680_REG_RAW_DATA_LEN     (BME680_REG_GAS_R_LSB_0 - BME680_REG_MEAS_STATUS_0 + 1)

// calibration data registers
#define BME680_REG_CD1_ADDR         0x89    // 25 byte calibration data
#define BME680_REG_CD1_LEN          25
#define BME680_REG_CD2_ADDR         0xe1    // 16 byte calibration data
#define BME680_REG_CD2_LEN          16
#define BME680_REG_CD3_ADDR         0x00    //  8 byte device specific calibration data
#define BME680_REG_CD3_LEN          8

// register structure definitions
#define BME680_NEW_DATA_BITS        0x80    // BME680_REG_MEAS_STATUS<7>
#define BME680_NEW_DATA_SHIFT       7       // BME680_REG_MEAS_STATUS<7>
#define BME680_GAS_MEASURING_BITS   0x40    // BME680_REG_MEAS_STATUS<6>
#define BME680_GAS_MEASURING_SHIFT  6       // BME680_REG_MEAS_STATUS<6>
#define BME680_MEASURING_BITS       0x20    // BME680_REG_MEAS_STATUS<5>
#define BME680_MEASURING_SHIFT      5       // BME680_REG_MEAS_STATUS<5>
#define BME680_GAS_MEAS_INDEX_BITS  0x0f    // BME680_REG_MEAS_STATUS<3:0>
#define BME680_GAS_MEAS_INDEX_SHIFT 0       // BME680_REG_MEAS_STATUS<3:0>

#define BME680_GAS_R_LSB_BITS       0xc0    // BME680_REG_GAS_R_LSB<7:6>
#define BME680_GAS_R_LSB_SHIFT      6       // BME680_REG_GAS_R_LSB<7:6>
#define BME680_GAS_VALID_BITS       0x20    // BME680_REG_GAS_R_LSB<5>
#define BME680_GAS_VALID_SHIFT      5       // BME680_REG_GAS_R_LSB<5>
#define BME680_HEAT_STAB_R_BITS     0x10    // BME680_REG_GAS_R_LSB<4>
#define BME680_HEAT_STAB_R_SHIFT    4       // BME680_REG_GAS_R_LSB<4>
#define BME680_GAS_RANGE_R_BITS     0x0f    // BME680_REG_GAS_R_LSB<3:0>
#define BME680_GAS_RANGE_R_SHIFT    0       // BME680_REG_GAS_R_LSB<3:0>

#define BME680_HEAT_OFF_BITS        0x04    // BME680_REG_CTRL_GAS_0<3>
#define BME680_HEAT_OFF_SHIFT       3       // BME680_REG_CTRL_GAS_0<3>

#define BME680_RUN_GAS_BITS         0x10    // BME680_REG_CTRL_GAS_1<4>
#define BME680_RUN_GAS_SHIFT        4       // BME680_REG_CTRL_GAS_1<4>
#define BME680_NB_CONV_BITS         0x0f    // BME680_REG_CTRL_GAS_1<3:0>
#define BME680_NB_CONV_SHIFT        0       // BME680_REG_CTRL_GAS_1<3:0>

#define BME680_SPI_3W_INT_EN_BITS   0x40    // BME680_REG_CTRL_HUM<6>
#define BME680_SPI_3W_INT_EN_SHIFT  6       // BME680_REG_CTRL_HUM<6>
#define BME680_OSR_H_BITS           0x07    // BME680_REG_CTRL_HUM<2:0>
#define BME680_OSR_H_SHIFT          0       // BME680_REG_CTRL_HUM<2:0>

#define BME680_OSR_T_BITS           0xe0    // BME680_REG_CTRL_MEAS<7:5>
#define BME680_OSR_T_SHIFT          5       // BME680_REG_CTRL_MEAS<7:5>
#define BME680_OSR_P_BITS           0x1c    // BME680_REG_CTRL_MEAS<4:2>
#define BME680_OSR_P_SHIFT          2       // BME680_REG_CTRL_MEAS<4:2>
#define BME680_MODE_BITS            0x03    // BME680_REG_CTRL_MEAS<1:0>
#define BME680_MODE_SHIFT           0       // BME680_REG_CTRL_MEAS<1:0>

#define BME680_FILTER_BITS          0x1c    // BME680_REG_CONFIG<4:2>
#define BME680_FILTER_SHIFT         2       // BME680_REG_CONFIG<4:2>
#define BME680_SPI_3W_EN_BITS       0x01    // BME680_REG_CONFIG<0>
#define BME680_SPI_3W_EN_SHIFT      0       // BME680_REG_CONFIG<0>

#define BME680_SPI_MEM_PAGE_BITS    0x10    // BME680_REG_STATUS<4>
#define BME680_SPI_MEM_PAGE_SHIFT   4       // BME680_REG_STATUS<4>

#define BME680_GAS_WAIT_BITS        0x3f    // BME680_REG_GAS_WAIT+x<5:0>
#define BME680_GAS_WAIT_SHIFT       0       // BME680_REG_GAS_WAIT+x<5:0>
#define BME680_GAS_WAIT_MULT_BITS   0xc0    // BME680_REG_GAS_WAIT+x<7:6>
#define BME680_GAS_WAIT_MULT_SHIFT  6       // BME680_REG_GAS_WAIT+x<7:6>

// commands
#define BME680_RESET_CMD            0xb6    // BME680_REG_RESET<7:0>
#define BME680_RESET_PERIOD         10      // reset time in ms

#define BME680_RHR_BITS             0x30    // BME680_REG_RES_HEAT_RANGE<5:4>
#define BME680_RHR_SHIFT            4       // BME680_REG_RES_HEAT_RANGE<5:4>
#define BME680_RSWE_BITS            0xf0    // BME680_REG_RANGE_SW_ERROR<7:4>
#define BME680_RSWE_SHIFT           4       // BME680_REG_RANGE_SW_ERROR<7:4>

// calibration data are stored in a calibration data map
#define BME680_CDM_SIZE (BME680_REG_CD1_LEN + BME680_REG_CD2_LEN + BME680_REG_CD3_LEN)
#define BME680_CDM_OFF1 0
#define BME680_CDM_OFF2 BME680_REG_CD1_LEN
#define BME680_CDM_OFF3 BME680_CDM_OFF2 + BME680_REG_CD2_LEN

// calibration parameter offsets in calibration data map
// calibration data from 0x89
#define BME680_CDM_T2   1
#define BME680_CDM_T3   3
#define BME680_CDM_P1   5
#define BME680_CDM_P2   7
#define BME680_CDM_P3   9
#define BME680_CDM_P4   11
#define BME680_CDM_P5   13
#define BME680_CDM_P7   15
#define BME680_CDM_P6   16
#define BME680_CDM_P8   19
#define BME680_CDM_P9   21
#define BME680_CDM_P10  23
// calibration data from 0e1
#define BME680_CDM_H2   25
#define BME680_CDM_H1   26
#define BME680_CDM_H3   28
#define BME680_CDM_H4   29
#define BME680_CDM_H5   30
#define BME680_CDM_H6   31
#define BME680_CDM_H7   32
#define BME680_CDM_T1   33
#define BME680_CDM_GH2  35
#define BME680_CDM_GH1  37
#define BME680_CDM_GH3  38
// device specific calibration data from 0x00
#define BME680_CDM_RHV  41      // 0x00 - res_heat_val
#define BME680_CDM_RHR  43      // 0x02 - res_heat_range
#define BME680_CDM_RSWE 45      // 0x04 - range_sw_error

static const char *TAG = "bme680";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

/**
 * @brief   Raw data (integer values) read from sensor
 */
typedef struct
{

    bool gas_valid;      // indicate that gas measurement results are valid
    bool heater_stable;  // indicate that heater temperature was stable

    uint32_t temperature;    // degree celsius x100
    uint32_t pressure;       // pressure in Pascal
    uint16_t humidity;       // relative humidity x1000 in %
    uint16_t gas_resistance; // gas resistance data
    uint8_t gas_range;      // gas resistance range

    uint8_t gas_index;      // heater profile used (0 ... 9)
    uint8_t meas_index;

} bme680_raw_data_t;

#define lsb_msb_to_type(t,b,o) (t)(((t)b[o+1] << 8) | b[o])
#define lsb_to_type(t,b,o)     (t)(b[o])
#define bme_set_reg_bit(byte, bitname, bit) ( (byte & ~bitname##_BITS) | \
                                              ((bit << bitname##_SHIFT) & bitname##_BITS) )
#define bme_get_reg_bit(byte, bitname)      ( (byte & bitname##_BITS) >> bitname##_SHIFT )

static inline esp_err_t read_reg_8_nolock(bme680_t *dev, uint8_t reg, uint8_t *data)
{
    return i2c_dev_read_reg(&dev->i2c_dev, reg, data, 1);
}

static inline esp_err_t write_reg_8_nolock(bme680_t *dev, uint8_t reg, uint8_t data)
{
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &data, 1);
}

static esp_err_t read_reg_8(bme680_t *dev, uint8_t reg, uint8_t *data)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8_nolock(dev, reg, data));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t bme680_set_mode(bme680_t *dev, uint8_t mode)
{
    uint8_t reg;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8_nolock(dev, BME680_REG_CTRL_MEAS, &reg));
    reg = bme_set_reg_bit(reg, BME680_MODE, mode);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8_nolock(dev, BME680_REG_CTRL_MEAS, reg));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

#define msb_lsb_xlsb_to_20bit(t,b,o) (t)((t) b[o] << 12 | (t) b[o+1] << 4 | b[o+2] >> 4)
#define msb_lsb_to_type(t,b,o)       (t)(((t)b[o] << 8) | b[o+1])

#define BME680_RAW_P_OFF BME680_REG_PRESS_MSB_0-BME680_REG_MEAS_STATUS_0
#define BME680_RAW_T_OFF (BME680_RAW_P_OFF + BME680_REG_TEMP_MSB_0 - BME680_REG_PRESS_MSB_0)
#define BME680_RAW_H_OFF (BME680_RAW_T_OFF + BME680_REG_HUM_MSB_0 - BME680_REG_TEMP_MSB_0)
#define BME680_RAW_G_OFF (BME680_RAW_H_OFF + BME680_REG_GAS_R_MSB_0 - BME680_REG_HUM_MSB_0)

static esp_err_t bme680_get_raw_data(bme680_t *dev, bme680_raw_data_t *raw_data)
{
    if (!dev->meas_started)
    {
        ESP_LOGE(TAG, "Measurement was not started");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[BME680_REG_RAW_DATA_LEN] = { 0 };

    if (!(dev->meas_status & BME680_NEW_DATA_BITS))
    {
        // read measurement status from sensor
        CHECK(read_reg_8(dev, BME680_REG_MEAS_STATUS_0, &dev->meas_status));
        // test whether there are new data
        if (!(dev->meas_status & BME680_NEW_DATA_BITS))
        {
            if (dev->meas_status & BME680_MEASURING_BITS)
            {
                ESP_LOGW(TAG, "Measurement is still running");
                return ESP_ERR_INVALID_STATE;
            }
            ESP_LOGW(TAG, "No new data");
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    dev->meas_started = false;
    raw_data->gas_index = dev->meas_status & BME680_GAS_MEAS_INDEX_BITS;

    // if there are new data, read raw data from sensor
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BME680_REG_RAW_DATA_0, raw, BME680_REG_RAW_DATA_LEN));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    raw_data->gas_valid     = bme_get_reg_bit(raw[BME680_RAW_G_OFF + 1], BME680_GAS_VALID);
    raw_data->heater_stable = bme_get_reg_bit(raw[BME680_RAW_G_OFF + 1], BME680_HEAT_STAB_R);

    raw_data->temperature    = msb_lsb_xlsb_to_20bit(uint32_t, raw, BME680_RAW_T_OFF);
    raw_data->pressure       = msb_lsb_xlsb_to_20bit(uint32_t, raw, BME680_RAW_P_OFF);
    raw_data->humidity       = msb_lsb_to_type(uint16_t, raw, BME680_RAW_H_OFF);
    raw_data->gas_resistance = ((uint16_t) raw[BME680_RAW_G_OFF] << 2) | raw[BME680_RAW_G_OFF + 1] >> 6;
    raw_data->gas_range      = raw[BME680_RAW_G_OFF + 1] & BME680_GAS_RANGE_R_BITS;

    /*
     * BME680_REG_MEAS_STATUS_1, BME680_REG_MEAS_STATUS_2
     * These data are not documented and it is not really clear when they are filled
     */
    ESP_LOGD(TAG, "Raw data: %" PRIu32 " %" PRIu32 " %d %d %d", raw_data->temperature, raw_data->pressure,
            raw_data->humidity, raw_data->gas_resistance, raw_data->gas_range);

    return ESP_OK;
}

/**
 * @brief   Calculate temperature from raw temperature value
 * @ref     BME280 datasheet, page 50
 */
static int16_t bme680_convert_temperature(bme680_t *dev, uint32_t raw_temperature)
{
    bme680_calib_data_t *cd = &dev->calib_data;

    int64_t var1;
    int64_t var2;
    int16_t temperature;

    var1 = ((((raw_temperature >> 3) - ((int32_t) cd->par_t1 << 1))) * ((int32_t) cd->par_t2)) >> 11;
    var2 = (((((raw_temperature >> 4) - ((int32_t) cd->par_t1)) * ((raw_temperature >> 4) - ((int32_t) cd->par_t1))) >> 12)
            * ((int32_t) cd->par_t3)) >> 14;
    cd->t_fine = (int32_t) (var1 + var2);
    temperature = (cd->t_fine * 5 + 128) >> 8;

    return temperature;
}

/**
 * @brief       Calculate pressure from raw pressure value
 * @copyright   Copyright (c) 2017 - 2018 Bosch Sensortec GmbH
 *
 * The algorithm was extracted from the original Bosch Sensortec BME680 driver
 * published as open source. Divisions and multiplications by potences of 2
 * were replaced by shift operations for effeciency reasons.
 *
 * @ref         [BME680_diver](https://github.com/BoschSensortec/BME680_driver)
 * @ref         BME280 datasheet, page 50
 */
static uint32_t bme680_convert_pressure(bme680_t *dev, uint32_t raw_pressure)
{
    bme680_calib_data_t *cd = &dev->calib_data;

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t pressure_comp;

    var1 = (((int32_t)cd->t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
            (int32_t)cd->par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)cd->par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)cd->par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
             ((int32_t)cd->par_p3 << 5)) >> 3) +
           (((int32_t)cd->par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)cd->par_p1) >> 15;
    pressure_comp = 1048576 - raw_pressure;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
    if (pressure_comp >= BME680_MAX_OVERFLOW_VAL)
        pressure_comp = ((pressure_comp / var1) << 1);
    else
        pressure_comp = ((pressure_comp << 1) / var1);
    var1 = ((int32_t)cd->par_p9 * (int32_t)(((pressure_comp >> 3) *
                                            (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) *
            (int32_t)cd->par_p8) >> 13;
    var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
            (int32_t)(pressure_comp >> 8) *
            (int32_t)cd->par_p10) >> 17;

    pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
                    ((int32_t)cd->par_p7 << 7)) >> 4);

    return (uint32_t)pressure_comp;
}

/**
 * @brief       Calculate humidty from raw humidity data
 * @copyright   Copyright (c) 2017 - 2018 Bosch Sensortec GmbH
 *
 * The algorithm was extracted from the original Bosch Sensortec BME680 driver
 * published as open source. Divisions and multiplications by potences of 2
 * were replaced by shift operations for effeciency reasons.
 *
 * @ref         [BME680_diver](https://github.com/BoschSensortec/BME680_driver)
 */
static uint32_t bme680_convert_humidity(bme680_t *dev, uint16_t raw_humidity)
{
    bme680_calib_data_t *cd = &dev->calib_data;

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t var6;
    int32_t temp_scaled;
    int32_t humidity;

    temp_scaled = (((int32_t) cd->t_fine * 5) + 128) >> 8;
    var1 = (int32_t) (raw_humidity - ((int32_t) ((int32_t) cd->par_h1 << 4)))
            - (((temp_scaled * (int32_t) cd->par_h3) / ((int32_t) 100)) >> 1);
    var2 = ((int32_t) cd->par_h2
            * (((temp_scaled * (int32_t) cd->par_h4) / ((int32_t) 100))
                    + (((temp_scaled * ((temp_scaled * (int32_t) cd->par_h5) / ((int32_t) 100))) >> 6) / ((int32_t) 100))
                    + (int32_t) (1 << 14))) >> 10;
    var3 = var1 * var2;
    var4 = (int32_t) cd->par_h6 << 7;
    var4 = ((var4) + ((temp_scaled * (int32_t) cd->par_h7) / ((int32_t) 100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    humidity = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

    if (humidity > 100000) /* Cap at 100%rH */
        humidity = 100000;
    else if (humidity < 0)
        humidity = 0;

    return (uint32_t) humidity;
}

/**
 * @brief   Lookup table for gas resitance computation
 * @ref     BME680 datasheet, page 19
 */
static float lookup_table[16][2] = {
        // const1, const2       // gas_range
        { 1.0,   8000000.0 },   // 0
        { 1.0,   4000000.0 },   // 1
        { 1.0,   2000000.0 },   // 2
        { 1.0,   1000000.0 },   // 3
        { 1.0,   499500.4995 }, // 4
        { 0.99,  248262.1648 }, // 5
        { 1.0,   125000.0 },    // 6
        { 0.992, 63004.03226 }, // 7
        { 1.0,   31281.28128 }, // 8
        { 1.0,   15625.0 },     // 9
        { 0.998, 7812.5 },      // 10
        { 0.995, 3906.25 },     // 11
        { 1.0,   1953.125 },    // 12
        { 0.99,  976.5625 },    // 13
        { 1.0,   488.28125 },   // 14
        { 1.0,   244.140625 }   // 15
};

/**
 * @brief   Calculate gas resistance from raw gas resitance value and gas range
 * @ref     BME680 datasheet
 */
static uint32_t bme680_convert_gas(bme680_t *dev, uint16_t gas, uint8_t gas_range)
{
    bme680_calib_data_t *cd = &dev->calib_data;

    float var1 = (1340.0 + 5.0 * cd->range_sw_err) * lookup_table[gas_range][0];
    return var1 * lookup_table[gas_range][1] / (gas - 512.0 + var1);
}

/**
 * @brief   Calculate internal duration representation
 *
 * Durations are internally representes as one byte
 *
 *  duration = value<5:0> * multiplier<7:6>
 *
 * where the multiplier is 1, 4, 16, or 64. Maximum duration is therefore
 * 64*64 = 4032 ms. The function takes a real world duration value given
 * in milliseconds and computes the internal representation.
 *
 * @ref Datasheet
 */
static uint8_t bme680_heater_duration(uint16_t duration)
{
    uint8_t multiplier = 0;

    while (duration > 63)
    {
        duration = duration / 4;
        multiplier++;
    }
    return (uint8_t) (duration | (multiplier << 6));
}

/**
 * @brief  Calculate internal heater resistance value from real temperature.
 *
 * @ref Datasheet of BME680
 */
static uint8_t bme680_heater_resistance(const bme680_t *dev, uint16_t temp)
{
    if (!dev)
        return 0;

    if (temp < BME680_HEATER_TEMP_MIN)
        temp = BME680_HEATER_TEMP_MIN;
    else if (temp > BME680_HEATER_TEMP_MAX)
        temp = BME680_HEATER_TEMP_MAX;

    const bme680_calib_data_t *cd = &dev->calib_data;

    // from datasheet
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    uint8_t res_heat_x;

    var1 = ((double) cd->par_gh1 / 16.0) + 49.0;
    var2 = (((double) cd->par_gh2 / 32768.0) * 0.0005) + 0.00235;
    var3 = (double) cd->par_gh3 / 1024.0;
    var4 = var1 * (1.0 + (var2 * (double) temp));
    var5 = var4 + (var3 * (double) dev->settings.ambient_temperature);
    res_heat_x = (uint8_t) (3.4
            * ((var5 * (4.0 / (4.0 + (double) cd->res_heat_range)) * (1.0 / (1.0 + ((double) cd->res_heat_val * 0.002)))) - 25));
    return res_heat_x;

}

///////////////////////////////////////////////////////////////////////////////

esp_err_t bme680_init_desc(bme680_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != BME680_I2C_ADDR_0 &&  addr != BME680_I2C_ADDR_1)
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

esp_err_t bme680_free_desc(bme680_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bme680_init_sensor(bme680_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    dev->meas_started = false;
    dev->meas_status = 0;
    dev->settings.ambient_temperature = 0;
    dev->settings.osr_temperature = BME680_OSR_NONE;
    dev->settings.osr_pressure = BME680_OSR_NONE;
    dev->settings.osr_humidity = BME680_OSR_NONE;
    dev->settings.filter_size = BME680_IIR_SIZE_0;
    dev->settings.heater_profile = BME680_HEATER_NOT_USED;
    memset(dev->settings.heater_temperature, 0, sizeof(uint16_t) * 10);
    memset(dev->settings.heater_duration, 0, sizeof(uint16_t) * 10);

    // reset the sensor
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8_nolock(dev, BME680_REG_RESET, BME680_RESET_CMD));
    vTaskDelay(pdMS_TO_TICKS(BME680_RESET_PERIOD));

    uint8_t chip_id = 0;
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8_nolock(dev, BME680_REG_ID, &chip_id));
    if (chip_id != 0x61)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        ESP_LOGE(TAG, "Chip id %02x is wrong, should be 0x61", chip_id);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t buf[BME680_CDM_SIZE];

    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BME680_REG_CD1_ADDR, buf + BME680_CDM_OFF1, BME680_REG_CD1_LEN));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BME680_REG_CD2_ADDR, buf + BME680_CDM_OFF2, BME680_REG_CD2_LEN));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BME680_REG_CD3_ADDR, buf + BME680_CDM_OFF3, BME680_REG_CD3_LEN));

    dev->calib_data.par_t1 = lsb_msb_to_type(uint16_t, buf, BME680_CDM_T1);
    dev->calib_data.par_t2 = lsb_msb_to_type(int16_t, buf, BME680_CDM_T2);
    dev->calib_data.par_t3 = lsb_to_type(int8_t, buf, BME680_CDM_T3);

    // pressure compensation parameters
    dev->calib_data.par_p1 = lsb_msb_to_type(uint16_t, buf, BME680_CDM_P1);
    dev->calib_data.par_p2 = lsb_msb_to_type(int16_t, buf, BME680_CDM_P2);
    dev->calib_data.par_p3 = lsb_to_type(int8_t, buf, BME680_CDM_P3);
    dev->calib_data.par_p4 = lsb_msb_to_type(int16_t, buf, BME680_CDM_P4);
    dev->calib_data.par_p5 = lsb_msb_to_type(int16_t, buf, BME680_CDM_P5);
    dev->calib_data.par_p6 = lsb_to_type(int8_t, buf, BME680_CDM_P6);
    dev->calib_data.par_p7 = lsb_to_type(int8_t, buf, BME680_CDM_P7);
    dev->calib_data.par_p8 = lsb_msb_to_type(int16_t, buf, BME680_CDM_P8);
    dev->calib_data.par_p9 = lsb_msb_to_type(int16_t, buf, BME680_CDM_P9);
    dev->calib_data.par_p10 = lsb_to_type(uint8_t, buf, BME680_CDM_P10);

    // humidity compensation parameters
    dev->calib_data.par_h1 = (uint16_t) (((uint16_t) buf[BME680_CDM_H1 + 1] << 4) | (buf[BME680_CDM_H1] & 0x0F));
    dev->calib_data.par_h2 = (uint16_t) (((uint16_t) buf[BME680_CDM_H2] << 4) | (buf[BME680_CDM_H2 + 1] >> 4));
    dev->calib_data.par_h3 = lsb_to_type(int8_t, buf, BME680_CDM_H3);
    dev->calib_data.par_h4 = lsb_to_type(int8_t, buf, BME680_CDM_H4);
    dev->calib_data.par_h5 = lsb_to_type(int8_t, buf, BME680_CDM_H5);
    dev->calib_data.par_h6 = lsb_to_type(uint8_t, buf, BME680_CDM_H6);
    dev->calib_data.par_h7 = lsb_to_type(int8_t, buf, BME680_CDM_H7);

    // gas sensor compensation parameters
    dev->calib_data.par_gh1 = lsb_to_type(int8_t, buf, BME680_CDM_GH1);
    dev->calib_data.par_gh2 = lsb_msb_to_type(int16_t, buf, BME680_CDM_GH2);
    dev->calib_data.par_gh3 = lsb_to_type(int8_t, buf, BME680_CDM_GH3);

    dev->calib_data.res_heat_range = (lsb_to_type(uint8_t, buf, BME680_CDM_RHR) & BME680_RHR_BITS) >> BME680_RHR_SHIFT;
    dev->calib_data.res_heat_val = (lsb_to_type(int8_t, buf, BME680_CDM_RHV));
    dev->calib_data.range_sw_err = (lsb_to_type(int8_t, buf, BME680_CDM_RSWE) & BME680_RSWE_BITS) >> BME680_RSWE_SHIFT;

    // Set ambient temperature of sensor to default value (25 degree C)
    dev->settings.ambient_temperature = 25;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    CHECK(bme680_set_oversampling_rates(dev, BME680_OSR_1X, BME680_OSR_1X, BME680_OSR_1X));
    CHECK(bme680_set_filter_size(dev, BME680_IIR_SIZE_3));

    // Set heater default profile 0 to 320 degree Celcius for 150 ms
    CHECK(bme680_set_heater_profile(dev, 0, 320, 150));
    CHECK(bme680_use_heater_profile(dev, 0));

    return ESP_OK;
}

esp_err_t bme680_force_measurement(bme680_t *dev)
{
    CHECK_ARG(dev);
    if (dev->meas_started)
    {
        ESP_LOGE(TAG, "Measurement is already running");
        return ESP_ERR_INVALID_STATE;
    }

    // Set the power mode to forced mode to trigger one TPHG measurement cycle
    CHECK_LOGE(bme680_set_mode(dev, BME680_FORCED_MODE),
            "Could not set forced mode to start TPHG measurement cycle");
    dev->meas_started = true;
    dev->meas_status = 0;

    ESP_LOGD(TAG, "Started measurement");

    return ESP_OK;
}

/**
 * @brief Estimate the measurement duration in RTOS ticks
 *
 * Timing formulas extracted from BME280 datasheet and test in some
 * experiments. They represent the maximum measurement duration.
 */
esp_err_t bme680_get_measurement_duration(const bme680_t *dev, uint32_t *duration)
{
    CHECK_ARG(dev && duration);

    *duration = 0; /* Calculate in us */

    // wake up duration from sleep into forced mode
    *duration += 1250;

    // THP cycle duration which consumes 1963 µs for each measurement at maximum
    if (dev->settings.osr_temperature)
        *duration += (1 << (dev->settings.osr_temperature - 1)) * 2300;
    if (dev->settings.osr_pressure)
        *duration += (1 << (dev->settings.osr_pressure - 1)) * 2300 + 575;
    if (dev->settings.osr_humidity)
        *duration += (1 << (dev->settings.osr_humidity - 1)) * 2300 + 575;

    // if gas measurement is used
    if (dev->settings.heater_profile != BME680_HEATER_NOT_USED && dev->settings.heater_duration[dev->settings.heater_profile]
            && dev->settings.heater_temperature[dev->settings.heater_profile])
    {
        // gas heating time
        *duration += dev->settings.heater_duration[dev->settings.heater_profile] * 1000;
        // gas measurement duration;
        *duration += 2300 + 575;
    }

    // round up to next ms (1 us ... 1000 us => 1 ms)
    *duration += 999;
    *duration /= 1000;

    // some ms tolerance
    *duration += 5;

    // ceil to next integer value that is divisible by portTICK_PERIOD_MS and
    // compute RTOS ticks (1 ... portTICK_PERIOD_MS =  1 tick)
    *duration = (*duration + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;

    // Since first RTOS tick can be shorter than the half of defined tick period,
    // the delay caused by vTaskDelay(duration) might be 1 or 2 ms shorter than
    // computed duration in rare cases. Since the duration is computed for maximum
    // and not for the typical durations and therefore tends to be too long, this
    // should not be a problem. Therefore, only one additional tick used.
    *duration += 1;

    return ESP_OK;
}

esp_err_t bme680_is_measuring(bme680_t *dev, bool *busy)
{
    CHECK_ARG(dev && busy);

    // if measurement wasn't started, it is of course not measuring
    if (!dev->meas_started)
    {
        *busy = false;
        return ESP_OK;
    }

    CHECK(read_reg_8(dev, BME680_REG_MEAS_STATUS_0, &dev->meas_status));
    *busy = dev->meas_status & BME680_MEASURING_BITS ? 1 : 0;

    return ESP_OK;
}

esp_err_t bme680_get_results_fixed(bme680_t *dev, bme680_values_fixed_t *results)
{
    CHECK_ARG(dev && results);

    // fill data structure with invalid values
    results->temperature = INT16_MIN;
    results->pressure = 0;
    results->humidity = 0;
    results->gas_resistance = 0;

    bme680_raw_data_t raw;
    CHECK(bme680_get_raw_data(dev, &raw));

    // use compensation algorithms to compute sensor values in fixed point format
    if (dev->settings.osr_temperature)
        results->temperature = bme680_convert_temperature(dev, raw.temperature);
    if (dev->settings.osr_pressure)
        results->pressure = bme680_convert_pressure(dev, raw.pressure);
    if (dev->settings.osr_humidity)
        results->humidity = bme680_convert_humidity(dev, raw.humidity);

    if (dev->settings.heater_profile != BME680_HEATER_NOT_USED)
    {
        // convert gas only if raw data are valid and heater was stable
        if (raw.gas_valid && raw.heater_stable)
            results->gas_resistance = bme680_convert_gas(dev, raw.gas_resistance, raw.gas_range);
        else if (!raw.gas_valid)
            ESP_LOGW(TAG, "Gas data is not valid");
        else
            ESP_LOGW(TAG, "Heater is not stable");
    }

    ESP_LOGD(TAG, "Fixed point sensor values - %d/100 deg.C, %" PRIu32 "/1000 %%, %" PRIu32 " Pa, %" PRIu32 " Ohm",
            results->temperature, results->humidity, results->pressure, results->gas_resistance);

    return ESP_OK;
}

esp_err_t bme680_get_results_float(bme680_t *dev, bme680_values_float_t *results)
{
    CHECK_ARG(dev && results);

    bme680_values_fixed_t fixed;
    CHECK(bme680_get_results_fixed(dev, &fixed));

    results->temperature = fixed.temperature / 100.0f;
    results->pressure = fixed.pressure / 100.0f;
    results->humidity = fixed.humidity / 1000.0f;
    results->gas_resistance = fixed.gas_resistance;

    return ESP_OK;
}

esp_err_t bme680_measure_fixed(bme680_t *dev, bme680_values_fixed_t *results)
{
    CHECK_ARG(dev && results);

    uint32_t duration;
    CHECK(bme680_get_measurement_duration(dev, &duration));
    if (duration == 0)
    {
        ESP_LOGE(TAG, "Failed to get measurement duration");
        return ESP_FAIL;
    }

    CHECK(bme680_force_measurement(dev));
    vTaskDelay(duration);

    return bme680_get_results_fixed(dev, results);
}

esp_err_t bme680_measure_float(bme680_t *dev, bme680_values_float_t *results)
{
    CHECK_ARG(dev && results);

    uint32_t duration;
    CHECK(bme680_get_measurement_duration(dev, &duration));
    if (duration == 0)
    {
        ESP_LOGE(TAG, "Failed to get measurement duration");
        return ESP_FAIL;
    }

    CHECK(bme680_force_measurement(dev));
    vTaskDelay(duration);

    return bme680_get_results_float(dev, results);
}

esp_err_t bme680_set_oversampling_rates(bme680_t *dev, bme680_oversampling_rate_t ost,
        bme680_oversampling_rate_t osp, bme680_oversampling_rate_t osh)
{
    CHECK_ARG(dev);

    bool ost_changed = dev->settings.osr_temperature != ost;
    bool osp_changed = dev->settings.osr_pressure != osp;
    bool osh_changed = dev->settings.osr_humidity != osh;

    if (!ost_changed && !osp_changed && !osh_changed)
        return ESP_OK;

    // Set the temperature, pressure and humidity oversampling
    dev->settings.osr_temperature = ost;
    dev->settings.osr_pressure = osp;
    dev->settings.osr_humidity = osh;

    uint8_t reg;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (ost_changed || osp_changed)
    {
        // read the current register value
        I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8_nolock(dev, BME680_REG_CTRL_MEAS, &reg));

        // set changed bit values
        if (ost_changed)
            reg = bme_set_reg_bit(reg, BME680_OSR_T, ost);
        if (osp_changed)
            reg = bme_set_reg_bit(reg, BME680_OSR_P, osp);

        // write back the new register value
        I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8_nolock(dev, BME680_REG_CTRL_MEAS, reg));
    }
    if (osh_changed)
    {
        // read the current register value
        I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8_nolock(dev, BME680_REG_CTRL_HUM, &reg));

        // set changed bit value
        reg = bme_set_reg_bit(reg, BME680_OSR_H, osh);

        // write back the new register value
        I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8_nolock(dev, BME680_REG_CTRL_HUM, reg));
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "Setting oversampling rates done: osrt=%d osp=%d osrh=%d",
            dev->settings.osr_temperature, dev->settings.osr_pressure, dev->settings.osr_humidity);

    return ESP_OK;
}

esp_err_t bme680_set_filter_size(bme680_t *dev, bme680_filter_size_t size)
{
    CHECK_ARG(dev);

    if (dev->settings.filter_size == size)
        return ESP_OK;

    /* Set the temperature, pressure and humidity settings */
    dev->settings.filter_size = size;

    uint8_t reg;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // read the current register value
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_8_nolock(dev, BME680_REG_CONFIG, &reg));
    // set changed bit value
    reg = bme_set_reg_bit(reg, BME680_FILTER, size);
    // write back the new register value
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8_nolock(dev, BME680_REG_CONFIG, reg));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "Setting filter size done: size=%d", dev->settings.filter_size);

    return ESP_OK;
}

esp_err_t bme680_set_heater_profile(bme680_t *dev, uint8_t profile, uint16_t temperature, uint16_t duration)
{
    CHECK_ARG(dev && profile < BME680_HEATER_PROFILES);

    bool temperature_changed = dev->settings.heater_temperature[profile] != temperature;
    bool duration_changed = dev->settings.heater_duration[profile] != duration;

    if (!temperature_changed && !duration_changed)
        return ESP_OK;

    // set external gas sensor configuration
    dev->settings.heater_temperature[profile] = temperature; // degree Celsius
    dev->settings.heater_duration[profile] = duration;       // milliseconds

    // compute internal gas sensor configuration parameters
    uint8_t heat_dur = bme680_heater_duration(duration);           // internal duration value
    uint8_t heat_res = bme680_heater_resistance(dev, temperature); // internal temperature value

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    // set internal gas sensor configuration parameters if changed
    if (temperature_changed)
        I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8_nolock(dev, BME680_REG_RES_HEAT_BASE + profile, heat_res));
    if (duration_changed)
        I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8_nolock(dev, BME680_REG_GAS_WAIT_BASE + profile, heat_dur));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);


    ESP_LOGD(TAG, "Setting heater profile %d done: temperature=%d duration=%d"
            " heater_resistance=%02x heater_duration=%02x", profile, dev->settings.heater_temperature[profile],
            dev->settings.heater_duration[profile], heat_dur, heat_res);

    return ESP_OK;
}

esp_err_t bme680_use_heater_profile(bme680_t *dev, int8_t profile)
{
    CHECK_ARG(dev);
    CHECK_ARG(profile >= -1 && profile < BME680_HEATER_PROFILES);

    if (dev->settings.heater_profile == profile)
        return ESP_OK;

    dev->settings.heater_profile = profile;

    uint8_t reg = 0; // set
    // set active profile
    reg = bme_set_reg_bit(reg, BME680_NB_CONV, profile != BME680_HEATER_NOT_USED ? profile : 0);

    // enable or disable gas measurement
    reg = bme_set_reg_bit(reg, BME680_RUN_GAS,
            (profile != BME680_HEATER_NOT_USED && dev->settings.heater_temperature[profile] && dev->settings.heater_duration[profile]));

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_8_nolock(dev, BME680_REG_CTRL_GAS_1, reg));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bme680_set_ambient_temperature(bme680_t *dev, int16_t ambient)
{
    CHECK_ARG(dev);

    if (dev->settings.ambient_temperature == ambient)
        return ESP_OK;

    // set ambient temperature configuration
    dev->settings.ambient_temperature = ambient; // degree Celsius

    // update all valid heater profiles
    uint8_t data[10];
    for (int i = 0; i < BME680_HEATER_PROFILES; i++)
        data[i] = dev->settings.heater_temperature[i]
                ? bme680_heater_resistance(dev, dev->settings.heater_temperature[i])
                : 0;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, BME680_REG_RES_HEAT_BASE, data, 10));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "Setting heater ambient temperature done: ambient=%d", dev->settings.ambient_temperature);

    return ESP_OK;
}

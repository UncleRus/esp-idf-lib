/*
 * Copyright (c) 2019 Ruslan V. Uss (https://github.com/UncleRus)
 * Copyright (c) 2022 m5stack (https://github.com/m5stack)
 * Copyright (c) 2024 vonguced (https://github.com/vonguced)
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
 * @file qmp6988.c
 *
 * ESP-IDF driver for QMP6988 digital temperature and pressure sensor
 *
 * Code based on m5stack <https://github.com/m5stack/M5Unit-ENV>\n
 * and Ruslan V. Uss <https://github.com/UncleRus>
 *
 * Copyright (c) 2019 Ruslan V. Uss (https://github.com/UncleRus)\n
 * Copyright (c) 2022 m5stack (https://github.com/m5stack)\n
 * Copyright (c) 2024 vonguced (https://github.com/vonguced)\n
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_idf_lib_helpers.h>
#include "qmp6988.h"

#define I2C_FREQ_HZ 1000000 // 1MHz

typedef uint8_t qmp6988_raw_data_t;

#define QMP6988_CHIP_ID 0x5C

#define QMP6988_CHIP_ID_REG         0xD1
#define QMP6988_RESET_REG           0xE0 /* Device reset register */
#define QMP6988_DEVICE_STAT_REG     0xF3 /* Device state register */
#define QMP6988_CTRLMEAS_REG        0xF4 /* Measurement Condition Control Register */
#define QMP6988_PRESSURE_MSB_REG    0xF7 /* Pressure MSB Register */
#define QMP6988_TEMPERATURE_MSB_REG 0xFA /* Temperature MSB Reg */

#define SUBTRACTOR 8388608

/* compensation calculation */
#define QMP6988_CALIBRATION_DATA_START  0xA0 /* QMP6988 compensation coefficients */
#define QMP6988_CALIBRATION_DATA_LENGTH 25

#define SHIFT_RIGHT_4_POSITION 4
#define SHIFT_LEFT_2_POSITION  2
#define SHIFT_LEFT_4_POSITION  4
#define SHIFT_LEFT_5_POSITION  5
#define SHIFT_LEFT_8_POSITION  8
#define SHIFT_LEFT_12_POSITION 12
#define SHIFT_LEFT_16_POSITION 16

#define QMP6988_CTRLMEAS_REG_MODE__POS 0
#define QMP6988_CTRLMEAS_REG_MODE__MSK 0x03
#define QMP6988_CTRLMEAS_REG_MODE__LEN 2

#define QMP6988_CTRLMEAS_REG_OSRST__POS 5
#define QMP6988_CTRLMEAS_REG_OSRST__MSK 0xE0
#define QMP6988_CTRLMEAS_REG_OSRST__LEN 3

#define QMP6988_CTRLMEAS_REG_OSRSP__POS 2
#define QMP6988_CTRLMEAS_REG_OSRSP__MSK 0x1C
#define QMP6988_CTRLMEAS_REG_OSRSP__LEN 3

#define QMP6988_CONFIG_REG             0xF1 /*IIR filter co-efficient setting Register*/
#define QMP6988_CONFIG_REG_FILTER__POS 0
#define QMP6988_CONFIG_REG_FILTER__MSK 0x07
#define QMP6988_CONFIG_REG_FILTER__LEN 3

/**
 * Structure holding raw calibration data for QMP6988.
 */
typedef struct _qmp6988_cali_data
{
    int32_t COE_a0;
    int16_t COE_a1;
    int16_t COE_a2;
    int32_t COE_b00;
    int16_t COE_bt1;
    int16_t COE_bt2;
    int16_t COE_bp1;
    int16_t COE_b11;
    int16_t COE_bp2;
    int16_t COE_b12;
    int16_t COE_b21;
    int16_t COE_bp3;
} qmp6988_cali_data_t;

static const char *TAG = "qmp6988";

// due to the fact that ticks can be smaller than portTICK_PERIOD_MS, one and
// a half tick period added to the duration to be sure that waiting time for
// the results is long enough
#define TIME_TO_TICKS(ms) (1 + ((ms) + (portTICK_PERIOD_MS - 1) + portTICK_PERIOD_MS / 2) / portTICK_PERIOD_MS)

#define CHECK(x)                                                                                                                                                                                       \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t __;                                                                                                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                                                                                                        \
            return __;                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    while (0)
#define CHECK_ARG(VAL)                                                                                                                                                                                 \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (!(VAL))                                                                                                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                                                                                                \
    }                                                                                                                                                                                                  \
    while (0)

static esp_err_t write_reg(qmp6988_t *dev, uint8_t out_reg, uint8_t cmd)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write(&dev->i2c_dev, &out_reg, sizeof(out_reg), &cmd, sizeof(cmd)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t qmp6988_read_reg(qmp6988_t *dev, uint8_t out_reg, qmp6988_raw_data_t *raw_data)
{
    CHECK(i2c_dev_read(&dev->i2c_dev, &out_reg, sizeof(uint8_t), raw_data, sizeof(qmp6988_raw_data_t)));
    return ESP_OK;
}

esp_err_t qmp6988_device_check(qmp6988_t *dev)
{
    qmp6988_raw_data_t chip_id = 0x00;
    qmp6988_read_reg(dev, QMP6988_CHIP_ID_REG, &chip_id);

    if (chip_id == QMP6988_CHIP_ID)
    {
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "QMP6988 chip id not matching. Expected: 0x%02X got: 0x%02X", QMP6988_CHIP_ID, chip_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
}

esp_err_t qmp6988_get_calibration_data(qmp6988_t *dev)
{
    uint8_t a_data_uint8_tr[QMP6988_CALIBRATION_DATA_LENGTH] = { 0 };
    qmp6988_cali_data_t qmp6988_cali;
    int len;

    for (len = 0; len < QMP6988_CALIBRATION_DATA_LENGTH; len += 1)
    {
        CHECK(qmp6988_read_reg(dev, QMP6988_CALIBRATION_DATA_START + len, &a_data_uint8_tr[len]));
    }

    qmp6988_cali.COE_a0 = (int32_t)(((int32_t)a_data_uint8_tr[18] << SHIFT_LEFT_12_POSITION) | (a_data_uint8_tr[19] << SHIFT_LEFT_4_POSITION) | (a_data_uint8_tr[24] & 0x0f)) << 12;
    qmp6988_cali.COE_a0 = qmp6988_cali.COE_a0 >> 12;

    qmp6988_cali.COE_a1 = (int16_t)((a_data_uint8_tr[20] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[21]);

    qmp6988_cali.COE_a2 = (int16_t)((a_data_uint8_tr[22] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[23]);

    qmp6988_cali.COE_b00
        = (int32_t)((((int32_t)a_data_uint8_tr[0] << SHIFT_LEFT_12_POSITION) | (a_data_uint8_tr[1] << SHIFT_LEFT_4_POSITION) | ((a_data_uint8_tr[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION))
                          << 12);
    qmp6988_cali.COE_b00 = qmp6988_cali.COE_b00 >> 12;

    qmp6988_cali.COE_bt1 = (int16_t)((a_data_uint8_tr[2] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[3]);

    qmp6988_cali.COE_bt2 = (int16_t)((a_data_uint8_tr[4] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[5]);

    qmp6988_cali.COE_bp1 = (int16_t)((a_data_uint8_tr[6] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[7]);

    qmp6988_cali.COE_b11 = (int16_t)((a_data_uint8_tr[8] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[9]);

    qmp6988_cali.COE_bp2 = (int16_t)((a_data_uint8_tr[10] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[11]);

    qmp6988_cali.COE_b12 = (int16_t)((a_data_uint8_tr[12] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[13]);

    qmp6988_cali.COE_b21 = (int16_t)((a_data_uint8_tr[14] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[15]);

    qmp6988_cali.COE_bp3 = (int16_t)((a_data_uint8_tr[16] << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[17]);

    dev->ik.a0 = qmp6988_cali.COE_a0;   // 20Q4
    dev->ik.b00 = qmp6988_cali.COE_b00; // 20Q4

    dev->ik.a1 = 3608L * (int32_t)qmp6988_cali.COE_a1 - 1731677965L; // 31Q23
    dev->ik.a2 = 16889L * (int32_t)qmp6988_cali.COE_a2 - 87619360L;  // 30Q47

    dev->ik.bt1 = 2982L * (int64_t)qmp6988_cali.COE_bt1 + 107370906L;   // 28Q15
    dev->ik.bt2 = 329854L * (int64_t)qmp6988_cali.COE_bt2 + 108083093L; // 34Q38
    dev->ik.bp1 = 19923L * (int64_t)qmp6988_cali.COE_bp1 + 1133836764L; // 31Q20
    dev->ik.b11 = 2406L * (int64_t)qmp6988_cali.COE_b11 + 118215883L;   // 28Q34
    dev->ik.bp2 = 3079L * (int64_t)qmp6988_cali.COE_bp2 - 181579595L;   // 29Q43
    dev->ik.b12 = 6846L * (int64_t)qmp6988_cali.COE_b12 + 85590281L;    // 29Q53
    dev->ik.b21 = 13836L * (int64_t)qmp6988_cali.COE_b21 + 79333336L;   // 29Q60
    dev->ik.bp3 = 2915L * (int64_t)qmp6988_cali.COE_bp3 + 157155561L;   // 28Q65
    return ESP_OK;
}

int16_t qmp6988_conv_Tx_02e(qmp6988_ik_data_t *ik, int32_t dt)
{
    int16_t ret;
    int64_t wk1, wk2;

    // wk1: 60Q4 // bit size
    wk1 = ((int64_t)ik->a1 * (int64_t)dt);       // 31Q23+24-1=54 (54Q23)
    wk2 = ((int64_t)ik->a2 * (int64_t)dt) >> 14; // 30Q47+24-1=53 (39Q33)
    wk2 = (wk2 * (int64_t)dt) >> 10;                   // 39Q33+24-1=62 (52Q23)
    wk2 = ((wk1 + wk2) / 32767) >> 19;                       // 54,52->55Q23 (20Q04)
    ret = (int16_t)((ik->a0 + wk2) >> 4);              // 21Q4 -> 17Q0
    return ret;
}

int32_t qmp6988_get_pressure_02e(qmp6988_ik_data_t *ik, int32_t dp, int16_t tx)
{
    int32_t ret;
    int64_t wk1, wk2, wk3;

    // wk1 = 48Q16 // bit size
    wk1 = ((int64_t)ik->bt1 * (int64_t)tx);       // 28Q15+16-1=43 (43Q15)
    wk2 = ((int64_t)ik->bp1 * (int64_t)dp) >> 5;  // 31Q20+24-1=54 (49Q15)
    wk1 += wk2;                                               // 43,49->50Q15
    wk2 = ((int64_t)ik->bt2 * (int64_t)tx) >> 1;  // 34Q38+16-1=49 (48Q37)
    wk2 = (wk2 * (int64_t)tx) >> 8;                     // 48Q37+16-1=63 (55Q29)
    wk3 = wk2;                                                // 55Q29
    wk2 = ((int64_t)ik->b11 * (int64_t)tx) >> 4;  // 28Q34+16-1=43 (39Q30)
    wk2 = (wk2 * (int64_t)dp) >> 1;                     // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                                               // 55,61->62Q29
    wk2 = ((int64_t)ik->bp2 * (int64_t)dp) >> 13; // 29Q43+24-1=52 (39Q30)
    wk2 = (wk2 * (int64_t)dp) >> 1;                     // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                                               // 62,61->63Q29
    wk1 += wk3 >> 14;                                         // Q29 >> 14 -> Q15
    wk2 = ((int64_t)ik->b12 * (int64_t)tx);       // 29Q53+16-1=45 (45Q53)
    wk2 = (wk2 * (int64_t)tx) >> 22;                    // 45Q53+16-1=61 (39Q31)
    wk2 = (wk2 * (int64_t)dp) >> 1;                     // 39Q31+24-1=62 (61Q30)
    wk3 = wk2;                                                // 61Q30
    wk2 = ((int64_t)ik->b21 * (int64_t)tx) >> 6;  // 29Q60+16-1=45 (39Q54)
    wk2 = (wk2 * (int64_t)dp) >> 23;                    // 39Q54+24-1=62 (39Q31)
    wk2 = (wk2 * (int64_t)dp) >> 1;                     // 39Q31+24-1=62 (61Q20)
    wk3 += wk2;                                               // 61,61->62Q30
    wk2 = ((int64_t)ik->bp3 * (int64_t)dp) >> 12; // 28Q65+24-1=51 (39Q53)
    wk2 = (wk2 * (int64_t)dp) >> 23;                    // 39Q53+24-1=62 (39Q30)
    wk2 = (wk2 * (int64_t)dp);                          // 39Q30+24-1=62 (62Q30)
    wk3 += wk2;                                               // 62,62->63Q30
    wk1 += wk3 >> 15;                                         // Q30 >> 15 = Q15
    wk1 /= 32767L;
    wk1 >>= 11;     // Q15 >> 7 = Q4
    wk1 += ik->b00; // Q4 + 20Q4
    // wk1 >>= 4; // 28Q4 -> 24Q0
    ret = (int32_t)wk1;
    return ret;
}

///////////////////////////////////////////////////////////////////////////

esp_err_t qmp6988_init_desc(qmp6988_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t qmp6988_free_desc(qmp6988_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t qmp6988_init(qmp6988_t *dev)
{
    CHECK_ARG(dev);

    dev->power_mode = QMP6988_SLEEP_MODE;
    dev->filter_mode = QMP6988_FILTERCOEFF_4;
    dev->oversampling_t_mode = QMP6988_OVERSAMPLING_1X;
    dev->oversampling_p_mode = QMP6988_OVERSAMPLING_8X;

    CHECK(qmp6988_device_check(dev));

    CHECK(write_reg(dev, QMP6988_RESET_REG, 0xe6));
    vTaskDelay(TIME_TO_TICKS(20)); // need to wait a short moment after reset
    CHECK(qmp6988_get_calibration_data(dev));
    CHECK(qmp6988_setup_powermode(dev, QMP6988_SLEEP_MODE));
    CHECK(qmp6988_set_filter(dev, QMP6988_FILTERCOEFF_4));
    CHECK(qmp6988_set_p_oversampling(dev, QMP6988_OVERSAMPLING_8X));
    CHECK(qmp6988_set_t_oversampling(dev, QMP6988_OVERSAMPLING_1X));

    return ESP_OK;
}

esp_err_t qmp6988_setup_powermode(qmp6988_t *dev, qmp6988_power_mode_t power_mode)
{
    CHECK_ARG(dev);

    uint8_t data = 0x00;

    dev->power_mode = power_mode;
    CHECK(qmp6988_read_reg(dev, QMP6988_CTRLMEAS_REG, &data));
    data &= 0xfc;
    data |= power_mode;
    CHECK(write_reg(dev, QMP6988_CTRLMEAS_REG, data));

    vTaskDelay(TIME_TO_TICKS(20));

    return ESP_OK;
}

esp_err_t qmp6988_set_filter(qmp6988_t *dev, qmp6988_filter_t filter_mode)
{
    CHECK_ARG(dev);

    dev->filter_mode = filter_mode;
    CHECK(write_reg(dev, QMP6988_CONFIG_REG, filter_mode));
    vTaskDelay(TIME_TO_TICKS(20));

    return ESP_OK;
}

esp_err_t qmp6988_set_p_oversampling(qmp6988_t *dev, qmp6988_oversampling_t oversampling_p_mode)
{
    CHECK_ARG(dev);

    qmp6988_raw_data_t data = 0x00;

    dev->oversampling_p_mode = oversampling_p_mode;
    CHECK(qmp6988_read_reg(dev, QMP6988_CTRLMEAS_REG, &data));
    data &= 0xe3;
    data |= (oversampling_p_mode << 2);
    CHECK(write_reg(dev, QMP6988_CTRLMEAS_REG, data));
    vTaskDelay(TIME_TO_TICKS(20));

    return ESP_OK;
}

esp_err_t qmp6988_set_t_oversampling(qmp6988_t *dev, qmp6988_oversampling_t oversampling_t_mode)
{
    CHECK_ARG(dev);

    qmp6988_raw_data_t data = 0x00;

    dev->oversampling_t_mode = oversampling_t_mode;
    CHECK(qmp6988_read_reg(dev, QMP6988_CTRLMEAS_REG, &data));
    data &= 0x1f;
    data |= (oversampling_t_mode << 5);
    CHECK(write_reg(dev, QMP6988_CTRLMEAS_REG, data));
    vTaskDelay(TIME_TO_TICKS(20));

    return ESP_OK;
}

esp_err_t qmp6988_calc_pressure(qmp6988_t *dev, float *p)
{
    CHECK_ARG(dev && p);

    uint32_t P_read, T_read;
    int32_t P_raw, T_raw;
    uint8_t a_data_uint8_tr[6] = { 0 };
    int32_t T_int, P_int;
    int len;

    // press
    for (len = 0; len < 3; len += 1)
    {
        CHECK(qmp6988_read_reg(dev, QMP6988_PRESSURE_MSB_REG + len, &a_data_uint8_tr[len]));
    }
    P_read = (uint32_t)((((uint32_t)(a_data_uint8_tr[0])) << SHIFT_LEFT_16_POSITION) | (((uint16_t)(a_data_uint8_tr[1])) << SHIFT_LEFT_8_POSITION) | (a_data_uint8_tr[2]));
    P_raw = (int32_t)(P_read - SUBTRACTOR);

    // temp
    for (len = 3; len < 6; len += 1)
    {
        CHECK(qmp6988_read_reg(dev, QMP6988_TEMPERATURE_MSB_REG + len - 3, &a_data_uint8_tr[len]));
    }
    T_read = (uint32_t)((((uint32_t)(a_data_uint8_tr[3])) << SHIFT_LEFT_16_POSITION) | (((uint16_t)(a_data_uint8_tr[4])) << SHIFT_LEFT_8_POSITION) | (a_data_uint8_tr[5]));
    T_raw = (int32_t)(T_read - SUBTRACTOR);

    T_int = qmp6988_conv_Tx_02e(&(dev->ik), T_raw);
    P_int = qmp6988_get_pressure_02e(&(dev->ik), P_raw, T_int);
    dev->temperature = (float)T_int / 256.0f;
    dev->pressure = (float)P_int / 16.0f;

    *p = dev->pressure;

    return ESP_OK;
}

esp_err_t qmp6988_calc_temperature(qmp6988_t *dev, float *t)
{
    CHECK_ARG(t);

    float dummy;
    CHECK(qmp6988_calc_pressure(dev, &dummy));
    *t = dev->temperature;

    return ESP_OK;
}

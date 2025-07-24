/*
 * Copyright (c) 2024 xyzroe <i@xyzroe.cc>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
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
 * @file qmi8658c.c
 * @brief QMI8658C sensor driver
 * @author xyzroe
 * ESP-IDF driver for QMI8658C sensor
 *
 * Datasheet: https://qstcorp.com/upload/pdf/202202/QMI8658C%20datasheet%20rev%200.9.pdf
 *
 * Copyright (c) 2024 xyzroe <i@xyzroe.cc>
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "qmi8658c.h"

static const char *TAG = "qmi8658c";

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

static qmi_ctx_t qmi_ctx;

inline static esp_err_t send_command_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return i2c_dev_write(dev, NULL, 0, data, 2);
}

static esp_err_t send_command(i2c_dev_t *dev, uint8_t reg, uint8_t value)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, send_command_nolock(dev, reg, value));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

#define REPLY_DELAY_MS 25

static esp_err_t read_register(i2c_dev_t *dev, uint8_t reg, uint8_t *value)
{
    I2C_DEV_TAKE_MUTEX(dev);

    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, value, 1));

    vTaskDelay(pdMS_TO_TICKS(REPLY_DELAY_MS));

    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t qmi8658c_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = QMI8658C_I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t qmi8658c_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(dev);
    /*
        uint8_t ctrl7, ctrl1;

        // Read QMI8658_CTRL7 register
        CHECK(read_register(dev, QMI8658_CTRL7, &ctrl7));

        // Disable accelerometer, gyroscope, magnetometer and attitude engine
        ctrl7 &= 0xF0;
        CHECK(send_command(dev, QMI8658_CTRL7, ctrl7));

        // Disable sensor by turning off the internal 2 MHz oscillator
        CHECK(read_register(dev, QMI8658_CTRL1, &ctrl1));
        ctrl1 |= (1 << 0);
        CHECK(send_command(dev, QMI8658_CTRL1, ctrl1));

        // Read these two registers again to verify
        CHECK(read_register(dev, QMI8658_CTRL7, &ctrl7));
        CHECK(read_register(dev, QMI8658_CTRL1, &ctrl1));

        // Verify if the sensor is properly disabled
        if (!(ctrl7 & 0x0F) && (ctrl1 & 0x01))
        {
            return i2c_dev_delete_mutex(dev);
        }
        else
        {
            return ESP_FAIL;
        }
        */
}

esp_err_t qmi8658c_reset(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return send_command(dev, QMI8658_RESET, 0xB0);
}

/* Accelerometer sensitivity table */
uint16_t acc_scale_sensitivity_table[4] = {
    ACC_SCALE_SENSITIVITY_2G, // Sensitivity for ±2g range.
    ACC_SCALE_SENSITIVITY_4G, // Sensitivity for ±4g range.
    ACC_SCALE_SENSITIVITY_8G, // Sensitivity for ±8g range.
    ACC_SCALE_SENSITIVITY_16G // Sensitivity for ±16g range.
};

/* Gyroscope sensitivity table */
uint16_t gyro_scale_sensitivity_table[8] = {
    GYRO_SCALE_SENSITIVITY_16DPS,   // Sensitivity for ±16 degrees per second range.
    GYRO_SCALE_SENSITIVITY_32DPS,   // Sensitivity for ±32 degrees per second range.
    GYRO_SCALE_SENSITIVITY_64DPS,   // Sensitivity for ±64 degrees per second range.
    GYRO_SCALE_SENSITIVITY_128DPS,  // Sensitivity for ±128 degrees per second range.
    GYRO_SCALE_SENSITIVITY_256DPS,  // Sensitivity for ±256 degrees per second range.
    GYRO_SCALE_SENSITIVITY_512DPS,  // Sensitivity for ±512 degrees per second range.
    GYRO_SCALE_SENSITIVITY_1024DPS, // Sensitivity for ±1024 degrees per second range.
    GYRO_SCALE_SENSITIVITY_2048DPS  // Sensitivity for ±2048 degrees per second range.
};

esp_err_t qmi8658c_setup(i2c_dev_t *dev, qmi8658c_config_t *config)
{
    CHECK_ARG(dev && config);

    // reset device
    qmi8658c_reset(dev);

    // set mode
    uint8_t ctrl7;
    CHECK(read_register(dev, QMI8658_CTRL7, &ctrl7));
    ctrl7 = (ctrl7 & 0xFC) | config->mode;
    CHECK(send_command(dev, QMI8658_CTRL7, ctrl7));

    // set accelerometr scale and ODR
    uint8_t ctrl2;
    CHECK(read_register(dev, QMI8658_CTRL2, &ctrl2));
    ctrl2 = (ctrl2 & 0xF0) | config->acc_odr;
    ctrl2 = (ctrl2 & 0x8F) | (config->acc_scale << 4);
    CHECK(send_command(dev, QMI8658_CTRL2, ctrl2));

    // set accelerometer scale and sensitivity
    qmi_ctx.acc_scale = config->acc_scale;
    qmi_ctx.acc_sensitivity = acc_scale_sensitivity_table[config->acc_scale];

    // set gyroscope scale and ODR
    uint8_t ctrl3;
    CHECK(read_register(dev, QMI8658_CTRL3, &ctrl3));
    ctrl3 = (ctrl3 & 0xF0) | config->gyro_odr;
    ctrl3 = (ctrl3 & 0x8F) | (config->gyro_scale << 4);
    CHECK(send_command(dev, QMI8658_CTRL3, ctrl3));

    // set gyroscope scale and sensitivity
    qmi_ctx.gyro_scale = config->gyro_scale;
    qmi_ctx.gyro_sensitivity = gyro_scale_sensitivity_table[config->gyro_scale];

    // read device ID and revision ID
    CHECK(read_register(dev, QMI8658_WHO_AM_I, &qmi_ctx.who_am_i));
    CHECK(read_register(dev, QMI8658_REVISION, &qmi_ctx.revision));

    ESP_LOGW(TAG, "device ID: 0x%02X, revision: 0x%02X", qmi_ctx.who_am_i, qmi_ctx.revision);

    // Verify mode setting
    uint8_t qmi8658_ctrl7;
    CHECK(read_register(dev, QMI8658_CTRL7, &qmi8658_ctrl7));
    if ((qmi8658_ctrl7 & 0x03) != config->mode)
    {
        ESP_LOGE(TAG, "Mode setting verification failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t qmi8658c_read_data(i2c_dev_t *dev, qmi8658c_data_t *data)
{
    CHECK_ARG(dev && data);

    // Read accelerometer data
    uint8_t acc_x_l, acc_x_h, acc_y_l, acc_y_h, acc_z_l, acc_z_h;
    CHECK(read_register(dev, QMI8658_ACC_X_L, &acc_x_l));
    CHECK(read_register(dev, QMI8658_ACC_X_H, &acc_x_h));
    CHECK(read_register(dev, QMI8658_ACC_Y_L, &acc_y_l));
    CHECK(read_register(dev, QMI8658_ACC_Y_H, &acc_y_h));
    CHECK(read_register(dev, QMI8658_ACC_Z_L, &acc_z_l));
    CHECK(read_register(dev, QMI8658_ACC_Z_H, &acc_z_h));

    // ESP_LOGE(TAG, "acc_x_l: %d, acc_x_h: %d, acc_y_l: %d, acc_y_h: %d, acc_z_l: %d, acc_z_h: %d", acc_x_l, acc_x_h, acc_y_l, acc_y_h, acc_z_l, acc_z_h);

    int16_t acc_x = (int16_t)((acc_x_h << 8) | acc_x_l);
    int16_t acc_y = (int16_t)((acc_y_h << 8) | acc_y_l);
    int16_t acc_z = (int16_t)((acc_z_h << 8) | acc_z_l);

    // ESP_LOGW(TAG, "acc_x: %d, acc_y: %d, acc_z: %d", acc_x, acc_y, acc_z);

    data->acc.x = (float)acc_x / qmi_ctx.acc_sensitivity;
    data->acc.y = (float)acc_y / qmi_ctx.acc_sensitivity;
    data->acc.z = (float)acc_z / qmi_ctx.acc_sensitivity;

    // Read gyroscope data
    uint8_t gyr_x_l, gyr_x_h, gyr_y_l, gyr_y_h, gyr_z_l, gyr_z_h;
    CHECK(read_register(dev, QMI8658_GYR_X_L, &gyr_x_l));
    CHECK(read_register(dev, QMI8658_GYR_X_H, &gyr_x_h));
    CHECK(read_register(dev, QMI8658_GYR_Y_L, &gyr_y_l));
    CHECK(read_register(dev, QMI8658_GYR_Y_H, &gyr_y_h));
    CHECK(read_register(dev, QMI8658_GYR_Z_L, &gyr_z_l));
    CHECK(read_register(dev, QMI8658_GYR_Z_H, &gyr_z_h));

    // ESP_LOGE(TAG, "gyr_x_l: %d, gyr_x_h: %d, gyr_y_l: %d, gyr_y_h: %d, gyr_z_l: %d, gyr_z_h: %d", gyr_x_l, gyr_x_h, gyr_y_l, gyr_y_h, gyr_z_l, gyr_z_h);

    int16_t gyr_x = (int16_t)((gyr_x_h << 8) | gyr_x_l);
    int16_t gyr_y = (int16_t)((gyr_y_h << 8) | gyr_y_l);
    int16_t gyr_z = (int16_t)((gyr_z_h << 8) | gyr_z_l);

    // ESP_LOGW(TAG, "gyr_x: %d, gyr_y: %d, gyr_z: %d", gyr_x, gyr_y, gyr_z);

    data->gyro.x = (float)gyr_x / qmi_ctx.gyro_sensitivity;
    data->gyro.y = (float)gyr_y / qmi_ctx.gyro_sensitivity;
    data->gyro.z = (float)gyr_z / qmi_ctx.gyro_sensitivity;

    // Read temperature data
    uint8_t temp_l, temp_h;
    CHECK(read_register(dev, QMI8658_TEMP_L, &temp_l));
    CHECK(read_register(dev, QMI8658_TEMP_H, &temp_h));

    // ESP_LOGE(TAG, "temp_l: %d, temp_h: %d", temp_l, temp_h);

    int16_t temp = (int16_t)((temp_h << 8) | temp_l);
    // ESP_LOGW(TAG, "temp: %d", temp);

    data->temperature = (float)temp / TEMPERATURE_SENSOR_RESOLUTION;

    // ESP_LOGW(TAG, "Acc: x=%f, y=%f, z=%f; Gyro: x=%f, y=%f, z=%f; Temp: %f",
    //          data->acc.x, data->acc.y, data->acc.z, data->gyro.x, data->gyro.y, data->gyro.z, data->temperature);

    return ESP_OK;
}
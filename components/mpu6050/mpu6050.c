
/*
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/* C headers */
#include <string.h>

/* esp headers */
#include <esp_utils.h>

/* mpu6050 headers */
#include "mpu6050.h"

#define TAG __FILENAME__

/**
 * MPU6050 internal registers definitions:
 */
#define MPU6050_REGISTER_XG_OFFS_TC         (0x00)
#define MPU6050_REGISTER_YG_OFFS_TC         (0x01)
#define MPU6050_REGISTER_ZG_OFFS_TC         (0x02)
#define MPU6050_REGISTER_X_FINE_GAIN        (0x03)
#define MPU6050_REGISTER_Y_FINE_GAIN        (0x04)
#define MPU6050_REGISTER_Z_FINE_GAIN        (0x05)
#define MPU6050_REGISTER_XA_OFFS_H          (0x06)
#define MPU6050_REGISTER_XA_OFFS_L_TC       (0x07)
#define MPU6050_REGISTER_YA_OFFS_H          (0x08)
#define MPU6050_REGISTER_YA_OFFS_L_TC       (0x09)
#define MPU6050_REGISTER_ZA_OFFS_H          (0x0A)
#define MPU6050_REGISTER_ZA_OFFS_L_TC       (0x0B)
#define MPU6050_REGISTER_SELF_TEST_X        (0x0D)
#define MPU6050_REGISTER_SELF_TEST_Y        (0x0E)
#define MPU6050_REGISTER_SELF_TEST_Z        (0x0F)
#define MPU6050_REGISTER_SELF_TEST_A        (0x10)
#define MPU6050_REGISTER_XG_OFFS_USRH       (0x13)
#define MPU6050_REGISTER_XG_OFFS_USRL       (0x14)
#define MPU6050_REGISTER_YG_OFFS_USRH       (0x15)
#define MPU6050_REGISTER_YG_OFFS_USRL       (0x16)
#define MPU6050_REGISTER_ZG_OFFS_USRH       (0x17)
#define MPU6050_REGISTER_ZG_OFFS_USRL       (0x18)
#define MPU6050_REGISTER_SMPLRT_DIV         (0x19)
#define MPU6050_REGISTER_CONFIG             (0x1A)
#define MPU6050_REGISTER_GYRO_CONFIG        (0x1B)
#define MPU6050_REGISTER_ACCEL_CONFIG       (0x1C)
#define MPU6050_REGISTER_FF_THR             (0x1D)
#define MPU6050_REGISTER_FF_DUR             (0x1E)
#define MPU6050_REGISTER_MOT_THR            (0x1F)
#define MPU6050_REGISTER_MOT_DUR            (0x20)
#define MPU6050_REGISTER_ZRMOT_THR          (0x21)
#define MPU6050_REGISTER_ZRMOT_DUR          (0x22)
#define MPU6050_REGISTER_FIFO_EN            (0x23)
#define MPU6050_REGISTER_I2C_MST_CTRL       (0x24)
#define MPU6050_REGISTER_I2C_SLV0_ADDR      (0x25)
#define MPU6050_REGISTER_I2C_SLV0_REG       (0x26)
#define MPU6050_REGISTER_I2C_SLV0_CTRL      (0x27)
#define MPU6050_REGISTER_I2C_SLV1_ADDR      (0x28)
#define MPU6050_REGISTER_I2C_SLV1_REG       (0x29)
#define MPU6050_REGISTER_I2C_SLV1_CTRL      (0x2A)
#define MPU6050_REGISTER_I2C_SLV2_ADDR      (0x2B)
#define MPU6050_REGISTER_I2C_SLV2_REG       (0x2C)
#define MPU6050_REGISTER_I2C_SLV2_CTRL      (0x2D)
#define MPU6050_REGISTER_I2C_SLV3_ADDR      (0x2E)
#define MPU6050_REGISTER_I2C_SLV3_REG       (0x2F)
#define MPU6050_REGISTER_I2C_SLV3_CTRL      (0x30)
#define MPU6050_REGISTER_I2C_SLV4_ADDR      (0x31)
#define MPU6050_REGISTER_I2C_SLV4_REG       (0x32)
#define MPU6050_REGISTER_I2C_SLV4_DO        (0x33)
#define MPU6050_REGISTER_I2C_SLV4_CTRL      (0x34)
#define MPU6050_REGISTER_I2C_SLV4_DI        (0x35)
#define MPU6050_REGISTER_I2C_MST_STATUS     (0x36)
#define MPU6050_REGISTER_INT_PIN_CFG        (0x37)
#define MPU6050_REGISTER_INT_ENABLE         (0x38)
#define MPU6050_REGISTER_DMP_INT_STATUS     (0x39)
#define MPU6050_REGISTER_INT_STATUS         (0x3A)
#define MPU6050_REGISTER_ACCEL_XOUT_H       (0x3B)
#define MPU6050_REGISTER_ACCEL_XOUT_L       (0x3C)
#define MPU6050_REGISTER_ACCEL_YOUT_H       (0x3D)
#define MPU6050_REGISTER_ACCEL_YOUT_L       (0x3E)
#define MPU6050_REGISTER_ACCEL_ZOUT_H       (0x3F)
#define MPU6050_REGISTER_ACCEL_ZOUT_L       (0x40)
#define MPU6050_REGISTER_TEMP_OUT_H         (0x41)
#define MPU6050_REGISTER_TEMP_OUT_L         (0x42)
#define MPU6050_REGISTER_GYRO_XOUT_H        (0x43)
#define MPU6050_REGISTER_GYRO_XOUT_L        (0x44)
#define MPU6050_REGISTER_GYRO_YOUT_H        (0x45)
#define MPU6050_REGISTER_GYRO_YOUT_L        (0x46)
#define MPU6050_REGISTER_GYRO_ZOUT_H        (0x47)
#define MPU6050_REGISTER_GYRO_ZOUT_L        (0x48)
#define MPU6050_REGISTER_EXT_SENS_DATA_00   (0x49)
#define MPU6050_REGISTER_EXT_SENS_DATA_01   (0x4A)
#define MPU6050_REGISTER_EXT_SENS_DATA_02   (0x4B)
#define MPU6050_REGISTER_EXT_SENS_DATA_03   (0x4C)
#define MPU6050_REGISTER_EXT_SENS_DATA_04   (0x4D)
#define MPU6050_REGISTER_EXT_SENS_DATA_05   (0x4E)
#define MPU6050_REGISTER_EXT_SENS_DATA_06   (0x4F)
#define MPU6050_REGISTER_EXT_SENS_DATA_07   (0x50)
#define MPU6050_REGISTER_EXT_SENS_DATA_08   (0x51)
#define MPU6050_REGISTER_EXT_SENS_DATA_09   (0x52)
#define MPU6050_REGISTER_EXT_SENS_DATA_10   (0x53)
#define MPU6050_REGISTER_EXT_SENS_DATA_11   (0x54)
#define MPU6050_REGISTER_EXT_SENS_DATA_12   (0x55)
#define MPU6050_REGISTER_EXT_SENS_DATA_13   (0x56)
#define MPU6050_REGISTER_EXT_SENS_DATA_14   (0x57)
#define MPU6050_REGISTER_EXT_SENS_DATA_15   (0x58)
#define MPU6050_REGISTER_EXT_SENS_DATA_16   (0x59)
#define MPU6050_REGISTER_EXT_SENS_DATA_17   (0x5A)
#define MPU6050_REGISTER_EXT_SENS_DATA_18   (0x5B)
#define MPU6050_REGISTER_EXT_SENS_DATA_19   (0x5C)
#define MPU6050_REGISTER_EXT_SENS_DATA_20   (0x5D)
#define MPU6050_REGISTER_EXT_SENS_DATA_21   (0x5E)
#define MPU6050_REGISTER_EXT_SENS_DATA_22   (0x5F)
#define MPU6050_REGISTER_EXT_SENS_DATA_23   (0x60)
#define MPU6050_REGISTER_MOT_DETECT_STATUS  (0x61)
#define MPU6050_REGISTER_I2C_SLV0_DO        (0x63)
#define MPU6050_REGISTER_I2C_SLV1_DO        (0x64)
#define MPU6050_REGISTER_I2C_SLV2_DO        (0x65)
#define MPU6050_REGISTER_I2C_SLV3_DO        (0x66)
#define MPU6050_REGISTER_I2C_MST_DELAY_CTRL (0x67)
#define MPU6050_REGISTER_SIGNAL_PATH_RESET  (0x68)
#define MPU6050_REGISTER_MOT_DETECT_CTRL    (0x69)
#define MPU6050_REGISTER_USER_CTRL          (0x6A)
#define MPU6050_REGISTER_PWR_MGMT_1         (0x6B)
#define MPU6050_REGISTER_PWR_MGMT_2         (0x6C)
#define MPU6050_REGISTER_BANK_SEL           (0x6D)
#define MPU6050_REGISTER_MEM_START_ADDR     (0x6E)
#define MPU6050_REGISTER_MEM_R_W            (0x6F)
#define MPU6050_REGISTER_DMP_CFG_1          (0x70)
#define MPU6050_REGISTER_DMP_CFG_2          (0x71)
#define MPU6050_REGISTER_FIFO_COUNTH        (0x72)
#define MPU6050_REGISTER_FIFO_COUNTL        (0x73)
#define MPU6050_REGISTER_FIFO_R_W           (0x74)
#define MPU6050_REGISTER_WHO_AM_I           (0x75)

#define PI              (3.14159265358979323846f)
#define GYRO_MEAS_ERROR (PI * (60.0f / 180.0f))
#define GYRO_MEAS_DRIFT (PI * (1.0f / 180.0f))
#define BETA            (sqrt(3.0f / 4.0f) * GYRO_MEAS_ERROR)
#define ZETA            (sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT)

static float quart[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static float delta_t = 0.0f;

/* ------------------------------------------------------ */
uint8_t mpu6050_get_aux_vddio_level(const mpu6050_config_t *setting)
{
    uint8_t level;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, &(level));
    return level;
}
/* ------------------------------------------------------ */
void mpu6050_set_aux_vddio_level(const mpu6050_config_t *setting, uint8_t level)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, level);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_rate(const mpu6050_config_t *setting)
{
    uint8_t rate;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_SMPLRT_DIV, &(rate));

    return rate;
}
/* ------------------------------------------------------ */
void mpu6050_set_rate(const mpu6050_config_t *setting, uint8_t rate)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_SMPLRT_DIV, rate);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_external_frame_sync(const mpu6050_config_t *setting)
{
    uint8_t frame;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH,
        &(frame));

    return frame;
}
/* ------------------------------------------------------ */
void mpu6050_set_external_frame_sync(const mpu6050_config_t *setting, uint8_t sync)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
        MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_dlpf_mode(const mpu6050_config_t *setting)
{
    uint8_t dlpf;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH,
        &(dlpf));

    return dlpf;
}
/* ------------------------------------------------------ */
void mpu6050_set_dlpf_mode(const mpu6050_config_t *setting, uint8_t mode)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_full_scale_gyro_range(const mpu6050_config_t *setting)
{
    uint8_t gyro_range;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
        MPU6050_GCONFIG_FS_SEL_LENGTH, &(gyro_range));
    return gyro_range;
}
/* ------------------------------------------------------ */
void mpu6050_set_full_scale_gyro_range(const mpu6050_config_t *setting, uint8_t range)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
        MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_accel_x_self_test_factory_trim(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_X, &buffer[0]);
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_A, &buffer[1]);

    return ((buffer[0] >> 3) | ((buffer[1] >> 4) & 0x03));
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_accel_y_self_test_factory_trim(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_Y, &buffer[0]);
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_A, &buffer[1]);

    return ((buffer[0] >> 3) | ((buffer[1] >> 2) & 0x03));
}
/* ---------------------------------------------------uint8_t byte = 0;--- */
uint8_t mpu6050_get_accel_z_self_test_factory_trim(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_SELF_TEST_Z, 2, (uint8_t *)buffer);

    return ((buffer[0] >> 3) | (buffer[1] & 0x03));
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_gyro_x_self_test_factory_trim(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_X, &(byte));

    return ((byte & 0x1F));
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_gyro_y_self_test_factory_trim(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_Y, &(byte));

    return ((byte & 0x1F));
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_gyro_z_self_test_factory_trim(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_Z, &(byte));

    return ((byte & 0x1F));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_x_self_test(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_accel_x_self_test(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_y_self_test(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_accel_y_self_test(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_z_self_test(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_accel_z_self_test(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, enabled);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_full_scale_accel_range(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
        MPU6050_ACONFIG_AFS_SEL_LENGTH, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_full_scale_accel_range(const mpu6050_config_t *setting, uint8_t range)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
        MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_dhpf_mode(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT,
        MPU6050_ACONFIG_ACCEL_HPF_LENGTH, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_dhpf_mode(const mpu6050_config_t *setting, uint8_t mode)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT,
        MPU6050_ACONFIG_ACCEL_HPF_LENGTH, mode);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_freefall_detection_threshold(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_FF_THR, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_freefall_detection_threshold(const mpu6050_config_t *setting, uint8_t threshold)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_FF_THR, threshold);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_freefall_detection_duration(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_FF_DUR, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_freefall_detection_duration(const mpu6050_config_t *setting, uint8_t duration)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_FF_DUR, duration);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_motion_detection_threshold(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_MOT_THR, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_motion_detection_threshold(const mpu6050_config_t *setting, uint8_t threshold)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_MOT_THR, threshold);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_motion_detection_duration(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_MOT_DUR, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_motion_detection_duration(const mpu6050_config_t *setting, uint8_t duration)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_MOT_DUR, duration);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_zero_motion_detection_threshold(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_ZRMOT_THR, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_zero_motion_detection_threshold(const mpu6050_config_t *setting, uint8_t threshold)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_ZRMOT_THR, threshold);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_zero_motion_detection_duration(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_ZRMOT_DUR, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_zero_motion_detection_duration(const mpu6050_config_t *setting, uint8_t duration)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_ZRMOT_DUR, duration);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_temp_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_temp_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_gyro_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_x_gyro_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_gyro_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_y_gyro_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_gyro_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_z_gyro_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_accel_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_2_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_2_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_1_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_1_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_0_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_0_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_multi_master_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_multi_master_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_wait_for_external_sensor_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_wait_for_external_sensor_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_3_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_3_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_read_write_transition_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_read_write_transition_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, enabled);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_master_clock_speed(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH,
        &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_master_clock_speed(const mpu6050_config_t *setting, uint8_t speed)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH,
        speed);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_address(const mpu6050_config_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 3)
        return (0);

    esp32_i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV0_ADDR + num * 3, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_address(const mpu6050_config_t *setting, uint8_t num, uint8_t address)
{
    if (num > 3)
        return;

    esp32_i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV0_ADDR + num * 3, address);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_register(const mpu6050_config_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 3)
        return (0);

    esp32_i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV0_REG + num * 3, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_register(const mpu6050_config_t *setting, uint8_t num, uint8_t reg)
{
    if (num > 3)
        return;

    esp32_i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV0_REG + num * 3, reg);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_enabled(const mpu6050_config_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 3)
        return (0);

    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_enabled(const mpu6050_config_t *setting, uint8_t num, bool enabled)
{
    if (num > 3)
        return;

    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_word_byte_swap(const mpu6050_config_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 3)
        return (0);

    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_BYTE_SW_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_word_byte_swap(const mpu6050_config_t *setting, uint8_t num, bool enabled)
{
    if (num > 3)
        return;

    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_BYTE_SW_BIT, enabled);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_write_mode(const mpu6050_config_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 3)
        return (0);

    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_REG_DIS_BIT, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_write_mode(const mpu6050_config_t *setting, uint8_t num, bool mode)
{
    if (num > 3)
        return;

    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_REG_DIS_BIT, mode);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_word_group_offset(const mpu6050_config_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 3)
        return (0);

    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_GRP_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_word_group_offset(const mpu6050_config_t *setting, uint8_t num, bool enabled)
{
    if (num > 3)
        return;

    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_GRP_BIT, enabled);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_data_length(const mpu6050_config_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 3)
        return (0);

    esp32_i2c_read_bits(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT,
        MPU6050_I2C_SLV_LEN_LENGTH, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_data_length(const mpu6050_config_t *setting, uint8_t num, uint8_t length)
{
    if (num > 3)
        return;

    esp32_i2c_write_bits(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT,
        MPU6050_I2C_SLV_LEN_LENGTH, length);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_4_address(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV4_ADDR, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_4_address(const mpu6050_config_t *setting, uint8_t address)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV4_ADDR, address);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_4_register(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV4_REG, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_4_register(const mpu6050_config_t *setting, uint8_t reg)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV4_REG, reg);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_4_output_byte(const mpu6050_config_t *setting, uint8_t data)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV4_DO, data);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_4_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_interrupt_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_4_interrupt_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_4_write_mode(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_4_write_mode(const mpu6050_config_t *setting, bool mode)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, mode);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_4_master_delay(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
        MPU6050_I2C_SLV4_MST_DLY_LENGTH, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_4_master_delay(const mpu6050_config_t *setting, uint8_t delay)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
        MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_4_input_byte(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV4_DI, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_passthrough_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_PASS_THROUGH_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_is_done(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_DONE_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_lost_arbitration(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_LOST_ARB_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_nack(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_NACK_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_3_nack(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV3_NACK_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_2_nack(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV2_NACK_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_1_nack(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV1_NACK_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_0_nack(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV0_NACK_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_interrupt_mode(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_interrupt_mode(const mpu6050_config_t *setting, bool mode)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_interrupt_drive(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_interrupt_drive(const mpu6050_config_t *setting, bool drive)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_interrupt_latch(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_interrupt_latch(const mpu6050_config_t *setting, bool latch)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_interrupt_latch_clear(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_interrupt_latch_clear(const mpu6050_config_t *setting, bool clear)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fsync_interrupt_level(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_fsync_interrupt_level(const mpu6050_config_t *setting, bool level)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fsync_interrupt_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_fsync_interrupt_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_i2c_bypass_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_i2c_bypass_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_clock_output_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_clock_output_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_int_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_INT_ENABLE, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_enabled(const mpu6050_config_t *setting, uint8_t enabled)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_INT_ENABLE, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_freefall_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_freefall_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_motion_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_motion_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_zero_motion_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_zero_motion_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_fifo_byte_overflow_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_fifo_byte_overflow_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_i2c_master_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_i2c_master_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_data_ready_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_data_ready_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_INT_STATUS, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_freefall_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_FF_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_motion_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_MOT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_zero_motion_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_ZMOT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_fifo_byte_overflow_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_i2c_master_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_I2C_MST_INT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_data_ready_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_get_acceleration(const mpu6050_config_t *setting, mpu6050_acceleration_t *data)
{
    uint8_t buffer[6];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_XOUT_H, 6, (uint8_t *)buffer);
    data->accel_x = (((int16_t)buffer[0]) << 8) | buffer[1];
    data->accel_y = (((int16_t)buffer[2]) << 8) | buffer[3];
    data->accel_z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_acceleration_x(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_XOUT_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_acceleration_y(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_YOUT_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_acceleration_z(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_ZOUT_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_temperature(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_TEMP_OUT_H, 2, buffer);

    int16_t rawtemp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return (rawtemp / 340.0) + 36.53;
}
/* ------------------------------------------------------ */
void mpu6050_get_rotation(const mpu6050_config_t *setting, mpu6050_rotation_t *data)
{
    uint8_t buffer[6];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_GYRO_XOUT_H, 6, buffer);
    data->gyro_x = (((int16_t)buffer[0]) << 8) | buffer[1];
    data->gyro_y = (((int16_t)buffer[2]) << 8) | buffer[3];
    data->gyro_z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_rotation_x(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_GYRO_XOUT_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_rotation_y(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_GYRO_YOUT_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_rotation_z(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_GYRO_ZOUT_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
void mpu6050_get_motion(const mpu6050_config_t *setting, mpu6050_acceleration_t *data_accel,
    mpu6050_rotation_t *data_gyro)
{
    uint8_t buffer[14];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_XOUT_H, 14, buffer);

    data_accel->accel_x = (((int16_t)buffer[0]) << 8) | buffer[1];
    data_accel->accel_y = (((int16_t)buffer[2]) << 8) | buffer[3];
    data_accel->accel_z = (((int16_t)buffer[4]) << 8) | buffer[5];
    data_gyro->gyro_x = (((int16_t)buffer[8]) << 8) | buffer[9];
    data_gyro->gyro_y = (((int16_t)buffer[10]) << 8) | buffer[11];
    data_gyro->gyro_z = (((int16_t)buffer[12]) << 8) | buffer[13];
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_external_sensor_byte(const mpu6050_config_t *setting, int position)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_EXT_SENS_DATA_00 + position, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
uint16_t mpu6050_get_external_sensor_word(const mpu6050_config_t *setting, int position)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_EXT_SENS_DATA_00 + position, 2, buffer);

    return ((((uint16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
uint32_t mpu6050_get_external_sensor_dword(const mpu6050_config_t *setting, int position)
{
    uint8_t buffer[4];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_EXT_SENS_DATA_00 + position, 4, buffer);

    return ((((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) | (((uint16_t)buffer[2]) << 8) | buffer[3]);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_motion_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_negative_motion_detected(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XNEG_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_positive_motion_detected(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XPOS_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_negative_motion_detected(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YNEG_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_positive_motion_detected(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YPOS_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_negative_motion_detected(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZNEG_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_positive_motion_detected(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZPOS_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_zero_motion_detected(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZRMOT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_output_byte(const mpu6050_config_t *setting, uint8_t num, uint8_t data)
{
    if (num > 3)
        return;

    esp32_i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV0_DO + num, data);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_external_shadow_delay_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_external_shadow_delay_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_delay_enabled(const mpu6050_config_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 4)
        return (0);

    esp32_i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, num, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_slave_delay_enabled(const mpu6050_config_t *setting, uint8_t num, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, num, enabled);
}
/* ------------------------------------------------------ */
void mpu6050_reset_gyroscope_path(const mpu6050_config_t *setting)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_GYRO_RESET_BIT, 1);
}
/* ------------------------------------------------------ */
void mpu6050_reset_accelerometer_path(const mpu6050_config_t *setting)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_ACCEL_RESET_BIT, 1);
}
/* ------------------------------------------------------ */
void mpu6050_reset_temperature_path(const mpu6050_config_t *setting)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_TEMP_RESET_BIT, 1);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_accelerometer_power_on_delay(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_DELAY_BIT,
        MPU6050_DETECT_ACCEL_DELAY_LENGTH, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_accelerometer_power_on_delay(const mpu6050_config_t *setting, uint8_t delay)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_DELAY_BIT,
        MPU6050_DETECT_ACCEL_DELAY_LENGTH, delay);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_freefall_detection_counter_decrement(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT,
        MPU6050_DETECT_FF_COUNT_LENGTH, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_freefall_detection_counter_decrement(const mpu6050_config_t *setting, uint8_t decrement)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT,
        MPU6050_DETECT_FF_COUNT_LENGTH, decrement);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_motion_detection_counter_decrement(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT,
        MPU6050_DETECT_MOT_COUNT_LENGTH, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_motion_detection_counter_decrement(const mpu6050_config_t *setting, uint8_t decrement)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT,
        MPU6050_DETECT_MOT_COUNT_LENGTH, decrement);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fifo_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_fifo_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_i2c_master_mode_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_i2c_master_mode_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void mpu6050_switch_spie_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled);
}
/* ------------------------------------------------------ */
void mpu6050_reset_fifo(const mpu6050_config_t *setting)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}
/* ------------------------------------------------------ */
void mpu6050_reset_sensors(const mpu6050_config_t *setting)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT, 1);
}
/* ------------------------------------------------------ */
void mpu6050_reset(const mpu6050_config_t *setting)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_sleep_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}

void mpu6050_set_sleep_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_wake_cycle_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_wake_cycle_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_temp_sensor_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_temp_sensor_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_clock_source(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH,
        &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_clock_source(const mpu6050_config_t *setting, uint8_t source)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH,
        source);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_wake_frequency(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT,
        MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_wake_frequency(const mpu6050_config_t *setting, uint8_t frequency)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT,
        MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_x_accel_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_standby_x_accel_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_y_accel_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_standby_y_accel_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_z_accel_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_standby_z_accel_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_x_gyro_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_standby_x_gyro_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_y_gyro_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_standby_y_gyro_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_z_gyro_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_standby_z_gyro_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, enabled);
}
/* ------------------------------------------------------ */
uint16_t mpu6050_get_fifo_count(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_FIFO_COUNTH, 2, buffer);

    return ((((uint16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_fifo_byte(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_FIFO_R_W, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_get_fifo_bytes(const mpu6050_config_t *setting, uint8_t *data, uint8_t length)
{
    if (length > 0)
    {
        esp32_i2c_read_bytes(setting, MPU6050_REGISTER_FIFO_R_W, length, data);
    }
    else
    {
        *data = 0;
    }
    return;
}
/* ------------------------------------------------------ */
void mpu6050_set_fifo_byte(const mpu6050_config_t *setting, uint8_t data)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_FIFO_R_W, data);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_device_id(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_device_id(const mpu6050_config_t *setting, uint8_t id)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_otp_bank_valid(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_otp_bank_valid(const mpu6050_config_t *setting, int8_t enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}
/* ------------------------------------------------------ */
int8_t mpu6050_get_x_gyro_offset_tc(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, &(byte));

    return ((int8_t)byte);
}
/* ------------------------------------------------------ */
void mpu6050_set_x_gyro_offset_tc(const mpu6050_config_t *setting, int8_t offset)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
/* ------------------------------------------------------ */
int8_t mpu6050_get_y_gyro_offset_tc(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, &(byte));

    return ((int8_t)byte);
}
/* ------------------------------------------------------ */
void mpu6050_set_y_gyro_offset_tc(const mpu6050_config_t *setting, int8_t offset)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
/* ------------------------------------------------------ */
int8_t mpu6050_get_z_gyro_offset_tc(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bits(setting, MPU6050_REGISTER_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, &(byte));

    return ((int8_t)byte);
}
/* ------------------------------------------------------ */
void mpu6050_set_z_gyro_offset_tc(const mpu6050_config_t *setting, int8_t offset)
{
    esp32_i2c_write_bits(setting, MPU6050_REGISTER_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
/* ------------------------------------------------------ */
int8_t mpu6050_get_x_fine_gain(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_X_FINE_GAIN, &(byte));

    return ((int8_t)byte);
}
/* ------------------------------------------------------ */
void mpu6050_set_x_fine_gain(const mpu6050_config_t *setting, int8_t gain)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_X_FINE_GAIN, gain);
}
/* ------------------------------------------------------ */
int8_t mpu6050_get_y_fine_gain(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_Y_FINE_GAIN, &(byte));

    return ((int8_t)byte);
    ;
}
/* ------------------------------------------------------ */
void mpu6050_set_y_fine_gain(const mpu6050_config_t *setting, int8_t gain)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_Y_FINE_GAIN, gain);
}
/* ------------------------------------------------------ */
int8_t mpu6050_get_z_fine_gain(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_Z_FINE_GAIN, &(byte));

    return ((int8_t)byte);
}
/* ------------------------------------------------------ */
void mpu6050_set_z_fine_gain(const mpu6050_config_t *setting, int8_t gain)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_Z_FINE_GAIN, gain);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_x_accel_offset(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_XA_OFFS_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
void mpu6050_set_x_accel_offset(const mpu6050_config_t *setting, int16_t offset)
{
    esp32_i2c_write_word(setting, MPU6050_REGISTER_XA_OFFS_H, offset);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_y_accel_offset(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_YA_OFFS_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
void mpu6050_set_y_accel_offset(const mpu6050_config_t *setting, int16_t offset)
{
    esp32_i2c_write_word(setting, MPU6050_REGISTER_YA_OFFS_H, offset);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_z_accel_offset(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_ZA_OFFS_H, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
void mpu6050_set_z_accel_offset(const mpu6050_config_t *setting, int16_t offset)
{
    esp32_i2c_write_word(setting, MPU6050_REGISTER_ZA_OFFS_H, offset);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_x_gyro_offset(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_XG_OFFS_USRH, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
void mpu6050_set_x_gyro_offset(const mpu6050_config_t *setting, int16_t offset)
{
    esp32_i2c_write_word(setting, MPU6050_REGISTER_XG_OFFS_USRH, offset);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_y_gyro_offset(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_YG_OFFS_USRH, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
void mpu6050_set_y_gyro_offset(const mpu6050_config_t *setting, int16_t offset)
{
    esp32_i2c_write_word(setting, MPU6050_REGISTER_YG_OFFS_USRH, offset);
}
/* ------------------------------------------------------ */
int16_t mpu6050_get_z_gyro_offset(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bytes(setting, MPU6050_REGISTER_ZG_OFFS_USRH, 2, buffer);

    return ((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/* ------------------------------------------------------ */
void mpu6050_set_z_gyro_offset(const mpu6050_config_t *setting, int16_t offset)
{
    esp32_i2c_write_word(setting, MPU6050_REGISTER_ZG_OFFS_USRH, offset);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_pll_ready_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_pll_ready_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_dmp_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_int_dmp_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_5_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_5_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_4_status(const mpu6050_config_t *setting)
{
    uint8_t buffer[2];
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_4_BIT, buffer);

    return ((buffer == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_3_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_3_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_2_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_2_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_1_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_1_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_0_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_0_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_ppl_ready_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_dmp_status(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_enabled(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, &(byte));

    return ((byte == 0x00) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
void mpu6050_set_dmp_enabled(const mpu6050_config_t *setting, bool enabled)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
void mpu6050_reset_dmp(const mpu6050_config_t *setting)
{
    esp32_i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_dmp_config_1(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_DMP_CFG_1, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_dmp_config_1(const mpu6050_config_t *setting, uint8_t config)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_DMP_CFG_1, config);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_dmp_config_2(const mpu6050_config_t *setting)
{
    uint8_t byte = 0;
    esp32_i2c_read_byte(setting, MPU6050_REGISTER_DMP_CFG_2, &(byte));

    return byte;
}
/* ------------------------------------------------------ */
void mpu6050_set_dmp_config_2(const mpu6050_config_t *setting, uint8_t config)
{
    esp32_i2c_write_byte(setting, MPU6050_REGISTER_DMP_CFG_2, config);
}
/* ------------------------------------------------------ */
float mpu6050_get_accel_res(uint8_t accel_scale)
{
    float accel_res = 0;

    switch (accel_scale)
    {
        case 0:
            accel_res = 2.0 / 32768.0;
            break;
        case 1:
            accel_res = 4.0 / 32768.0;
            break;
        case 2:
            accel_res = 8.0 / 32768.0;
            break;
        case 3:
            accel_res = 16.0 / 32768.0;
            break;
    }

    return (accel_res);
}
/* ------------------------------------------------------ */
float mpu6050_get_gyro_res(uint8_t gyro_scale)
{
    float gyro_res = 0;

    switch (gyro_scale)
    {
        case 0:
            gyro_res = 250.0 / 32768.0;
            break;
        case 1:
            gyro_res = 500.0 / 32768.0;
            break;
        case 2:
            gyro_res = 1000.0 / 32768.0;
            break;
        case 3:
            gyro_res = 2000.0 / 32768.0;
            break;
    }

    return (gyro_res);
}
/* ------------------------------------------------------ */
void mpu6050_calibrate(const mpu6050_config_t *setting, float *accel_bias_res, float *gyro_bias_res)
{
    int32_t accel_bias[3] = { 0, 0, 0 };
    int32_t gyro_bias[3] = { 0, 0, 0 };
    int32_t accel_bias_reg[3] = { 0, 0, 0 };
    uint16_t accel_temp[3] = { 0, 0, 0 };
    uint16_t gyro_temp[3] = { 0, 0, 0 };
    uint8_t mask_bit[3] = { 0, 0, 0 };
    uint32_t mask = 1uL;
    uint16_t gyro_sensitivity = 131;
    uint16_t accel_sensitivity = 16384;
    uint8_t tmp_data[12];
    uint16_t packet_count;

    mpu6050_reset(setting);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    mpu6050_set_clock_source(setting, MPU6050_CLOCK_PLL_XGYRO);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Configure device for bias calculation:
    mpu6050_set_int_enabled(setting, 0);
    mpu6050_set_fifo_enabled(setting, 0);
    mpu6050_set_accel_fifo_enabled(setting, 0);
    mpu6050_set_z_gyro_fifo_enabled(setting, 0);
    mpu6050_set_y_gyro_fifo_enabled(setting, 0);
    mpu6050_set_x_gyro_fifo_enabled(setting, 0);
    mpu6050_set_temp_fifo_enabled(setting, 0);
    mpu6050_set_clock_source(setting, MPU6050_CLOCK_INTERNAL);
    mpu6050_set_multi_master_enabled(setting, 0);
    mpu6050_set_fifo_enabled(setting, 0);
    mpu6050_set_i2c_master_mode_enabled(setting, 0);
    mpu6050_reset_sensors(setting);
    vTaskDelay(15 / portTICK_PERIOD_MS);

    // Configure MPU6050 gyro and accelerometer for bias calculation:
    mpu6050_set_rate(setting, 0x00); // Set sample rate to 1 kHz.
    mpu6050_set_dlpf_mode(setting, MPU6050_DLPF_BW_188);
    mpu6050_set_full_scale_accel_range(setting, MPU6050_ACCEL_FULL_SCALE_RANGE_2);
    mpu6050_set_full_scale_gyro_range(setting, MPU6050_GYRO_FULL_SCALE_RANGE_250);

    /**
     * Configure FIFO to capture data for bias calculation.
     */

    // Enable gyroscope and accelerometer sensors for FIFO:
    mpu6050_set_fifo_enabled(setting, 1);
    mpu6050_set_accel_fifo_enabled(setting, 1);
    mpu6050_set_z_gyro_fifo_enabled(setting, 1);
    mpu6050_set_y_gyro_fifo_enabled(setting, 1);
    mpu6050_set_x_gyro_fifo_enabled(setting, 1);
    vTaskDelay(80 / portTICK_PERIOD_MS); // Accumulate 80 samples in 80 ms.

    // At end of sample accumulation, turn off FIFO sensor read:
    mpu6050_set_fifo_enabled(setting, 0);
    mpu6050_set_accel_fifo_enabled(setting, 0);
    mpu6050_set_z_gyro_fifo_enabled(setting, 0);
    mpu6050_set_y_gyro_fifo_enabled(setting, 0);
    mpu6050_set_x_gyro_fifo_enabled(setting, 0);
    mpu6050_set_temp_fifo_enabled(setting, 0);

    // Sets of full gyro and accelerometer data for averaging:
    packet_count = mpu6050_get_fifo_count(setting) / 12;

    for (int i = 0; i < packet_count; i++)
    {
        // Read data for averaging:
        mpu6050_get_fifo_bytes(setting, &tmp_data[0], 6);
        accel_temp[0] = (int16_t)(((int16_t)tmp_data[0] << 8) | tmp_data[1]);
        accel_temp[1] = (int16_t)(((int16_t)tmp_data[2] << 8) | tmp_data[3]);
        accel_temp[2] = (int16_t)(((int16_t)tmp_data[4] << 8) | tmp_data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)tmp_data[6] << 8) | tmp_data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)tmp_data[8] << 8) | tmp_data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)tmp_data[10] << 8) | tmp_data[11]);

        // Sum individual 16-bit biases to get accumulated signed 32-bit biases:
        accel_bias[0] += (int32_t)accel_temp[0];
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }

    // Normalize sums to get average count biases:
    accel_bias[0] /= (int32_t)packet_count;
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    // Remove gravity from the z-axis accelerometer bias calculation:
    if (accel_bias[2] > 0L)
        accel_bias[2] -= (int32_t)accel_sensitivity;
    else
        accel_bias[2] += (int32_t)accel_sensitivity;

    /**
     * Construct the gyro biases for push to the hardware gyro bias registers,
     * which are reset to zero upon device startup:
     */

    // Divide by 4 to get 32.9 LSB per deg/s to expected bias input format.
    // Biases are additive, so change sign on calculated average gyro biases.
    tmp_data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
    tmp_data[1] = (-gyro_bias[0] / 4) & 0xFF;
    tmp_data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    tmp_data[3] = (-gyro_bias[1] / 4) & 0xFF;
    tmp_data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    tmp_data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers:
    mpu6050_set_x_gyro_offset(setting, ((int16_t)tmp_data[0]) << 8 | tmp_data[1]);
    mpu6050_set_y_gyro_offset(setting, ((int16_t)tmp_data[2]) << 8 | tmp_data[3]);
    mpu6050_set_z_gyro_offset(setting, ((int16_t)tmp_data[4]) << 8 | tmp_data[5]);

    // Construct gyro bias in deg/s for later manual subtraction:
    gyro_bias_res[0] = (float)gyro_bias[0] / (float)gyro_sensitivity;
    gyro_bias_res[1] = (float)gyro_bias[1] / (float)gyro_sensitivity;
    gyro_bias_res[2] = (float)gyro_bias[2] / (float)gyro_sensitivity;

    /**
     * Construct the accelerometer biases for push to the hardware accelerometer
     * bias registers. These registers contain factory trim values which must be
     * added to the calculated accelerometer biases; on boot up these registers
     * will hold non-zero values. In addition, bit 0 of the lower byte must be
     * preserved since it is used for temperature compensation calculations.
     * Accelerometer bias registers expect bias input as 2048 LSB per g, so that
     * the accelerometer biases calculated above must be divided by 8.
     */

    // Read factory accelerometer trim values:
    tmp_data[0] = mpu6050_get_x_accel_offset(setting);
    tmp_data[1] = mpu6050_get_y_accel_offset(setting);
    tmp_data[2] = mpu6050_get_z_accel_offset(setting);

    for (int i = 0; i < 3; i++)
    {
        // If temperature compensation bit is set, record that in mask_bit:
        if (accel_bias_reg[i] & mask)
            mask_bit[i] = 0x01;
    }

    /**
     * Construct total accelerometer bias, including calculated average
     * accelerometer bias from above (Subtract calculated averaged accelerometer
     * bias scaled to 2048 LSB/g (16g full scale).
     */

    accel_bias_reg[0] -= (accel_bias[0] / 8);
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    tmp_data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    tmp_data[1] = (accel_bias_reg[0]) & 0xFF;
    tmp_data[1] = tmp_data[1] | mask_bit[0];
    tmp_data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    tmp_data[3] = (accel_bias_reg[1]) & 0xFF;
    tmp_data[3] = tmp_data[3] | mask_bit[1];
    tmp_data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    tmp_data[5] = (accel_bias_reg[2]) & 0xFF;
    tmp_data[5] = tmp_data[5] | mask_bit[2];

    // Push accelerometer biases to hardware registers:
    mpu6050_set_x_accel_offset(setting, ((int16_t)tmp_data[0]) << 8 | tmp_data[1]);
    mpu6050_set_y_accel_offset(setting, ((int16_t)tmp_data[2]) << 8 | tmp_data[3]);
    mpu6050_set_z_accel_offset(setting, ((int16_t)tmp_data[4]) << 8 | tmp_data[5]);

    // Output scaled accelerometer biases for subtraction in the main program:
    accel_bias_res[0] = (float)accel_bias[0] / (float)accel_sensitivity;
    accel_bias_res[1] = (float)accel_bias[1] / (float)accel_sensitivity;
    accel_bias_res[2] = (float)accel_bias[2] / (float)accel_sensitivity;
}
/* ------------------------------------------------------ */
void mpu6050_self_test(const mpu6050_config_t *setting, float *destination)
{
    uint8_t self_test[6];
    float factory_trim[6];

    // Configure the accelerometer for self-test:
    mpu6050_set_accel_x_self_test(setting, true);
    mpu6050_set_accel_y_self_test(setting, true);
    mpu6050_set_accel_z_self_test(setting, true);
    mpu6050_set_full_scale_accel_range(setting, MPU6050_ACCEL_FULL_SCALE_RANGE_8);
    mpu6050_set_full_scale_gyro_range(setting, MPU6050_GYRO_FULL_SCALE_RANGE_250);

    self_test[0] = mpu6050_get_accel_x_self_test_factory_trim(setting);
    self_test[1] = mpu6050_get_accel_y_self_test_factory_trim(setting);
    self_test[2] = mpu6050_get_accel_z_self_test_factory_trim(setting);
    self_test[3] = mpu6050_get_gyro_x_self_test_factory_trim(setting);
    self_test[4] = mpu6050_get_gyro_y_self_test_factory_trim(setting);
    self_test[5] = mpu6050_get_gyro_z_self_test_factory_trim(setting);

    // Process results to allow final comparison with factory set values:
    factory_trim[0] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((self_test[0] - 1.0f) / 30.0f)));
    factory_trim[1] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((self_test[1] - 1.0f) / 30.0f)));
    factory_trim[2] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((self_test[2] - 1.0f) / 30.0f)));
    factory_trim[3] = (25.0f * 131.0f) * (pow(1.046f, (self_test[3] - 1.0f)));
    factory_trim[4] = (-25.0f * 131.0f) * (pow(1.046f, (self_test[4] - 1.0f)));
    factory_trim[5] = (25.0f * 131.0f) * (pow(1.046f, (self_test[5] - 1.0f)));

    // Report results as a ratio of "(STR - FT) / FT" (The change from Factory
    // Trim of the Self-Test Response).
    // To get to percent, must multiply by 100 and subtract result from 100.
    for (int i = 0; i < 6; i++)
        destination[i] = 100.0f + 100.0f * (self_test[i] - factory_trim[i]) / factory_trim[i];
}
/* ------------------------------------------------------ */
void mpu6050_madgwick_quaternion_update(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y,
    float gyro_z)
{
    float func_1, func_2, func_3;
    float j_11o24, j_12o23, j_13o22, j_14o21, j_32, j_33;
    float q_dot_1, q_dot_2, q_dot_3, q_dot_4;
    float hat_dot_1, hat_dot_2, hat_dot_3, hat_dot_4;
    float gyro_x_err, gyro_y_err, gyro_z_err;
    float gyro_x_bias, gyro_y_bias, gyro_z_bias;
    float norm;

    float half_q1 = 0.5f * quart[0];
    float half_q2 = 0.5f * quart[1];
    float half_q3 = 0.5f * quart[2];
    float half_q4 = 0.5f * quart[3];
    float double_q1 = 2.0f * quart[0];
    float double_q2 = 2.0f * quart[1];
    float double_q3 = 2.0f * quart[2];
    float double_q4 = 2.0f * quart[3];

    // Normalise accelerometer measurement:
    norm = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

    // Handle NaN:
    if (norm == 0.0f)
        return;

    norm = 1.0f / norm;
    accel_x *= norm;
    accel_y *= norm;
    accel_z *= norm;

    // Compute the objective function and Jacobian:
    func_1 = double_q2 * quart[3] - double_q1 * quart[2] - accel_x;
    func_2 = double_q1 * quart[1] + double_q3 * quart[3] - accel_y;
    func_3 = 1.0f - double_q2 * quart[3] - double_q3 * quart[2] - accel_z;
    j_11o24 = double_q3;
    j_12o23 = double_q4;
    j_13o22 = double_q1;
    j_14o21 = double_q2;
    j_32 = 2.0f * j_14o21;
    j_33 = 2.0f * j_11o24;

    // Compute the gradient (matrix multiplication):
    hat_dot_1 = j_14o21 * func_2 - j_11o24 * func_1;
    hat_dot_2 = j_12o23 * func_1 + j_13o22 * func_2 - j_32 * func_3;
    hat_dot_3 = j_12o23 * func_2 - j_33 * func_3 - j_13o22 * func_1;
    hat_dot_4 = j_14o21 * func_1 + j_11o24 * func_2;

    // Normalize the gradient:
    norm = sqrt(hat_dot_1 * hat_dot_1 + hat_dot_2 * hat_dot_2 + hat_dot_3 * hat_dot_3 + hat_dot_4 * hat_dot_4);
    hat_dot_1 /= norm;
    hat_dot_2 /= norm;
    hat_dot_3 /= norm;
    hat_dot_4 /= norm;

    // Compute estimated gyroscope biases:
    gyro_x_err = double_q1 * hat_dot_2 - double_q2 * hat_dot_1 - double_q3 * hat_dot_4 + double_q4 * hat_dot_3;
    gyro_y_err = double_q1 * hat_dot_3 + double_q2 * hat_dot_4 - double_q3 * hat_dot_1 - double_q4 * hat_dot_2;
    gyro_z_err = double_q1 * hat_dot_4 - double_q2 * hat_dot_3 + double_q3 * hat_dot_2 - double_q4 * hat_dot_1;

    // Compute and remove gyroscope biases:
    gyro_x_bias += gyro_x_err * delta_t * ZETA;
    gyro_y_bias += gyro_y_err * delta_t * ZETA;
    gyro_z_bias += gyro_z_err * delta_t * ZETA;

    // Compute the quaternion derivative:
    q_dot_1 = -half_q2 * gyro_x - half_q3 * gyro_y - half_q4 * gyro_z;
    q_dot_2 = half_q1 * gyro_x + half_q3 * gyro_z - half_q4 * gyro_y;
    q_dot_3 = half_q1 * gyro_y - half_q2 * gyro_z + half_q4 * gyro_x;
    q_dot_4 = half_q1 * gyro_z + half_q2 * gyro_y - half_q3 * gyro_x;

    // Compute then integrate estimated quaternion derivative:
    quart[0] += (q_dot_1 - (BETA * hat_dot_1)) * delta_t;
    quart[1] += (q_dot_2 - (BETA * hat_dot_2)) * delta_t;
    quart[2] += (q_dot_3 - (BETA * hat_dot_3)) * delta_t;
    quart[3] += (q_dot_4 - (BETA * hat_dot_4)) * delta_t;

    // Normalize the quaternion:
    norm = sqrt(quart[0] * quart[0] + quart[1] * quart[1] + quart[2] * quart[2] + quart[3] * quart[3]);
    norm = 1.0f / norm;
    quart[0] *= norm;
    quart[1] *= norm;
    quart[2] *= norm;
    quart[3] *= norm;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_test_connection(const mpu6050_config_t *setting)
{
    return ((mpu6050_get_device_id(setting) == 0x34) ? ESP_OK : ESP_FAIL);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_init(mpu6050_config_t *mpu6050_config)
{
    ESP_PARAM_CHECK(mpu6050_config);

    esp_err_t ret = ESP_OK;

    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    ret = i2c_dev_probe(mpu6050_config, I2C_DEV_READ);
    ESP_ERROR_RETURN(ret != ESP_OK, ret, "device not available");

#if defined(AUTO_CONFIG_ADDR)
    mpu6050_config->addr = mpu_6050_get_device_addr();
    ESP_ERROR_RETURN(mpu6050_config->addr == I2C_DEVICE_ADDR_NONE, ESP_FAIL, "failed to retrive i2c addr.");
#endif /* AUTO_CONFIG_ADDR */

    ret = i2c_dev_create_mutex(mpu6050_config);
    ESP_ERROR_RETURN(ret != ESP_OK, ret, "failed to create mutex for i2c dev.");

    mpu6050_set_clock_source(mpu6050_config, MPU6050_CLOCK_PLL_XGYRO);
    mpu6050_set_full_scale_gyro_range(mpu6050_config, MPU6050_GYRO_FULL_SCALE_RANGE_250);
    mpu6050_set_full_scale_accel_range(mpu6050_config, MPU6050_ACCEL_FULL_SCALE_RANGE_2);
    mpu6050_set_sleep_enabled(mpu6050_config, 0);

    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_deinit(mpu6050_config_t *mpu6050_config)
{
    ESP_PARAM_CHECK(mpu6050_config);

    ESP_ERROR_CHECK(i2cdev_done());

    ESP_ERROR_CHECK(i2c_dev_delete_mutex(mpu6050_config));
    return ESP_OK;
}
/* ------------------------------------------------------ */
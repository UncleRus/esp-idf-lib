/*
 * Copyright (c) 2022 Jan Veeh (jan.veeh@motius.de)
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
 * @file icm42670.c
 *
 * ESP-IDF driver for TDK ICM-42670-P IMU (found on ESP-RS board)
 *
 * Copyright (c) 2022 Jan Veeh (jan.veeh@motius.de)
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "icm42670.h"

#define I2C_FREQ_HZ 1000000 // 1MHz

static const char *TAG = "icm42670";


// register structure definitions
#define ICM42670_MCLK_RDY_BITS                      0x04    // ICM42670_REG_MCLK_RDY<3>
#define ICM42670_MCLK_RDY_SHIFT                     3       // ICM42670_REG_MCLK_RDY<3>

#define ICM42670_SPI_AP_4WIRE_BITS                  0x04    // ICM42670_REG_DEVICE_CONFIG<2>
#define ICM42670_SPI_AP_4WIRE_SHIFT                 2       // ICM42670_REG_DEVICE_CONFIG<2>
#define ICM42670_SPI_MODE_BITS                      0x01    // ICM42670_REG_DEVICE_CONFIG<0>
#define ICM42670_SPI_MODE_SHIFT                     0       // ICM42670_REG_DEVICE_CONFIG<0>

#define ICM42670_SOFT_RESET_DEVICE_CONFIG_BITS      0x10    // ICM42670_REG_SIGNAL_PATH_RESET<4>
#define ICM42670_SOFT_RESET_DEVICE_CONFIG_SHIFT     4       // ICM42670_REG_SIGNAL_PATH_RESET<4>
#define ICM42670_FIFO_FLUSH_BITS                    0x04    // ICM42670_REG_SIGNAL_PATH_RESET<2>
#define ICM42670_FIFO_FLUSH_SHIFT                   2       // ICM42670_REG_SIGNAL_PATH_RESET<2>

#define ICM42670_I3C_DDR_SLEW_RATE_BITS             0x38    // ICM42670_REG_DRIVE_CONFIG1<5:3>
#define ICM42670_I3C_DDR_SLEW_RATE_SHIFT            3       // ICM42670_REG_DRIVE_CONFIG1<5:3>
#define ICM42670_I3C_SDR_SLEW_RATE_BITS             0x07    // ICM42670_REG_DRIVE_CONFIG1<2:0>
#define ICM42670_I3C_SDR_SLEW_RATE_SHIFT            0       // ICM42670_REG_DRIVE_CONFIG1<2:0>

#define ICM42670_I2C_DDR_SLEW_RATE_BITS             0x38    // ICM42670_REG_DRIVE_CONFIG2<5:3>
#define ICM42670_I2C_DDR_SLEW_RATE_SHIFT            3       // ICM42670_REG_DRIVE_CONFIG2<5:3>
#define ICM42670_I2C_SDR_SLEW_RATE_BITS             0x07    // ICM42670_REG_DRIVE_CONFIG2<2:0>
#define ICM42670_I2C_SDR_SLEW_RATE_SHIFT            0       // ICM42670_REG_DRIVE_CONFIG2<2:0>

#define ICM42670_SPI_SLEW_RATE_BITS                 0x07    // ICM42670_REG_DRIVE_CONFIG3<2:0>
#define ICM42670_SPI_SLEW_RATE_SHIFT                0       // ICM42670_REG_DRIVE_CONFIG3<2:0>

#define ICM42670_INT2_MODE_BITS                     0x20    // ICM42670_REG_INT_CONFIG<5>
#define ICM42670_INT2_MODE_SHIFT                    5       // ICM42670_REG_INT_CONFIG<5>
#define ICM42670_INT2_DRIVE_CIRCUIT_BITS            0x10    // ICM42670_REG_INT_CONFIG<4>
#define ICM42670_INT2_DRIVE_CIRCUIT_SHIFT           4       // ICM42670_REG_INT_CONFIG<4>
#define ICM42670_INT2_POLARITY_BITS                 0x08    // ICM42670_REG_INT_CONFIG<3>
#define ICM42670_INT2_POLARITY_SHIFT                3       // ICM42670_REG_INT_CONFIG<3>
#define ICM42670_INT1_MODE_BITS                     0x04    // ICM42670_REG_INT_CONFIG<2>
#define ICM42670_INT1_MODE_SHIFT                    2       // ICM42670_REG_INT_CONFIG<2>
#define ICM42670_INT1_DRIVE_CIRCUIT_BITS            0x02    // ICM42670_REG_INT_CONFIG<1>
#define ICM42670_INT1_DRIVE_CIRCUIT_SHIFT           1       // ICM42670_REG_INT_CONFIG<1>
#define ICM42670_INT1_POLARITY_BITS                 0x01    // ICM42670_REG_INT_CONFIG<0>
#define ICM42670_INT1_POLARITY_SHIFT                0       // ICM42670_REG_INT_CONFIG<0>

#define ICM42670_ACCEL_LP_CLK_SEL_BITS              0x80    // ICM42670_REG_PWR_MGMT0<7>
#define ICM42670_ACCEL_LP_CLK_SEL_SHIFT             7       // ICM42670_REG_PWR_MGMT0<7>
#define ICM42670_IDLE_BITS                          0x10    // ICM42670_REG_PWR_MGMT0<4>
#define ICM42670_IDLE_SHIFT                         4       // ICM42670_REG_PWR_MGMT0<4>
#define ICM42670_GYRO_MODE_BITS                     0x0C    // ICM42670_REG_PWR_MGMT0<3:2>
#define ICM42670_GYRO_MODE_SHIFT                    2       // ICM42670_REG_PWR_MGMT0<3:2>
#define ICM42670_ACCEL_MODE_BITS                    0x03    // ICM42670_REG_PWR_MGMT0<1:0>
#define ICM42670_ACCEL_MODE_SHIFT                   0       // ICM42670_REG_PWR_MGMT0<1:0>

#define ICM42670_GYRO_UI_FS_SEL_BITS                0x60    // ICM42670_REG_GYRO_CONFIG0<6:5>
#define ICM42670_GYRO_UI_FS_SEL_SHIFT               5       // ICM42670_REG_GYRO_CONFIG0<6:5>
#define ICM42670_GYRO_ODR_BITS                      0x0F    // ICM42670_REG_GYRO_CONFIG0<3:0>
#define ICM42670_GYRO_ODR_SHIFT                     0       // ICM42670_REG_GYRO_CONFIG0<3:0>

#define ICM42670_ACCEL_UI_FS_SEL_BITS               0x60    // ICM42670_REG_ACCEL_CONFIG0<6:5>
#define ICM42670_ACCEL_UI_FS_SEL_SHIFT              5       // ICM42670_REG_ACCEL_CONFIG0<6:5>
#define ICM42670_ACCEL_ODR_BITS                     0x0F    // ICM42670_REG_ACCEL_CONFIG0<3:0>
#define ICM42670_ACCEL_ODR_SHIFT                    0       // ICM42670_REG_ACCEL_CONFIG0<3:0>

#define ICM42670_TEMP_FILT_BW_BITS                  0x70    // ICM42670_REG_TEMP_CONFIG0<6:4>
#define ICM42670_TEMP_FILT_BW_SHIFT                 4       // ICM42670_REG_TEMP_CONFIG0<6:4>

#define ICM42670_GYRO_UI_FILT_BW_BITS               0x07    // ICM42670_REG_GYRO_CONFIG1<2:0>
#define ICM42670_GYRO_UI_FILT_BW_SHIFT              0       // ICM42670_REG_GYRO_CONFIG1<2:0>

#define ICM42670_ACCEL_UI_AVG_BITS                  0x70    // ICM42670_REG_ACCEL_CONFIG1<6:4>
#define ICM42670_ACCEL_UI_AVG_SHIFT                 4       // ICM42670_REG_ACCEL_CONFIG1<6:4>
#define ICM42670_ACCEL_UI_FILT_BW_BITS              0x07    // ICM42670_REG_ACCEL_CONFIG1<2:0>
#define ICM42670_ACCEL_UI_FILT_BW_SHIFT             0       // ICM42670_REG_ACCEL_CONFIG1<2:0>

#define ICM42670_DMP_POWER_SAVE_EN_BITS             0x08    // ICM42670_REG_APEX_CONFIG0<3>
#define ICM42670_DMP_POWER_SAVE_EN_SHIFT            3       // ICM42670_REG_APEX_CONFIG0<3>
#define ICM42670_DMP_INIT_EN_BITS                   0x04    // ICM42670_REG_APEX_CONFIG0<2>
#define ICM42670_DMP_INIT_EN_SHIFT                  2       // ICM42670_REG_APEX_CONFIG0<2>
#define ICM42670_DMP_MEM_RESET_EN_BITS              0x01    // ICM42670_REG_APEX_CONFIG0<0>
#define ICM42670_DMP_MEM_RESET_EN_SHIFT             0       // ICM42670_REG_APEX_CONFIG0<0>

#define ICM42670_SMD_ENABLE_BITS                    0x40    // ICM42670_REG_APEX_CONFIG1<6>
#define ICM42670_SMD_ENABLE_SHIFT                   6       // ICM42670_REG_APEX_CONFIG1<6>
#define ICM42670_FF_ENABLE_BITS                     0x20    // ICM42670_REG_APEX_CONFIG1<5>
#define ICM42670_FF_ENABLE_SHIFT                    5       // ICM42670_REG_APEX_CONFIG1<5>
#define ICM42670_TILT_ENABLE_BITS                   0x10    // ICM42670_REG_APEX_CONFIG1<4>
#define ICM42670_TILT_ENABLE_SHIFT                  4       // ICM42670_REG_APEX_CONFIG1<4>
#define ICM42670_PED_ENABLE_BITS                    0x08    // ICM42670_REG_APEX_CONFIG1<3>
#define ICM42670_PED_ENABLE_SHIFT                   3       // ICM42670_REG_APEX_CONFIG1<3>
#define ICM42670_DMP_ODR_BITS                       0x03    // ICM42670_REG_APEX_CONFIG1<1:0>
#define ICM42670_DMP_ODR_SHIFT                      0       // ICM42670_REG_APEX_CONFIG1<1:0>

#define ICM42670_WOM_INT_DUR_BITS                   0x18    // ICM42670_REG_WOM_CONFIG<4:3>
#define ICM42670_WOM_INT_DUR_SHIFT                  3       // ICM42670_REG_WOM_CONFIG<4:3>
#define ICM42670_WOM_INT_MODE_BITS                  0x04    // ICM42670_REG_WOM_CONFIG<2>
#define ICM42670_WOM_INT_MODE_SHIFT                 2       // ICM42670_REG_WOM_CONFIG<2>
#define ICM42670_WOM_MODE_BITS                      0x02    // ICM42670_REG_WOM_CONFIG<1>
#define ICM42670_WOM_MODE_SHIFT                     1       // ICM42670_REG_WOM_CONFIG<1>
#define ICM42670_WOM_EN_BITS                        0x01    // ICM42670_REG_WOM_CONFIG<0>
#define ICM42670_WOM_EN_SHIFT                       0       // ICM42670_REG_WOM_CONFIG<0>

#define ICM42670_FIFO_MODE_BITS                     0x02    // ICM42670_REG_FIFO_CONFIG1<1>
#define ICM42670_FIFO_MODE_SHIFT                    1       // ICM42670_REG_FIFO_CONFIG1<1>
#define ICM42670_FIFO_BYPASS_BITS                   0x01    // ICM42670_REG_FIFO_CONFIG1<0>
#define ICM42670_FIFO_BYPASS_SHIFT                  0       // ICM42670_REG_FIFO_CONFIG1<0>

#define ICM42670_ST_INT1_EN_BITS                    0x80    // ICM42670_REG_INT_SOURCE0<7>
#define ICM42670_ST_INT1_EN_SHIFT                   7       // ICM42670_REG_INT_SOURCE0<7>
#define ICM42670_FSYNC_INT1_EN_BITS                 0x40    // ICM42670_REG_INT_SOURCE0<6>
#define ICM42670_FSYNC_INT1_EN_SHIFT                6       // ICM42670_REG_INT_SOURCE0<6>
#define ICM42670_PLL_RDY_INT1_EN_BITS               0x20    // ICM42670_REG_INT_SOURCE0<5>
#define ICM42670_PLL_RDY_INT1_EN_SHIFT              5       // ICM42670_REG_INT_SOURCE0<5>
#define ICM42670_RESET_DONE_INT1_EN_BITS            0x10    // ICM42670_REG_INT_SOURCE0<4>
#define ICM42670_RESET_DONE_INT1_EN_SHIFT           4       // ICM42670_REG_INT_SOURCE0<4>
#define ICM42670_DRDY_INT1_EN_BITS                  0x08    // ICM42670_REG_INT_SOURCE0<3>
#define ICM42670_DRDY_INT1_EN_SHIFT                 3       // ICM42670_REG_INT_SOURCE0<3>
#define ICM42670_FIFO_THS_INT1_EN_BITS              0x04    // ICM42670_REG_INT_SOURCE0<2>
#define ICM42670_FIFO_THS_INT1_EN_SHIFT             2       // ICM42670_REG_INT_SOURCE0<2>
#define ICM42670_FIFO_FULL_INT1_EN_BITS             0x02    // ICM42670_REG_INT_SOURCE0<1>
#define ICM42670_FIFO_FULL_INT1_EN_SHIFT            1       // ICM42670_REG_INT_SOURCE0<1>
#define ICM42670_AGC_RDY_INT1_EN_BITS               0x01    // ICM42670_REG_INT_SOURCE0<0>
#define ICM42670_AGC_RDY_INT1_EN_SHIFT              0       // ICM42670_REG_INT_SOURCE0<0>

#define ICM42670_I3C_PROTOCOL_ERROR_INT1_EN_BITS    0x40    // ICM42670_REG_INT_SOURCE1<6>
#define ICM42670_I3C_PROTOCOL_ERROR_INT1_EN_SHIFT   6       // ICM42670_REG_INT_SOURCE1<6>
#define ICM42670_SMD_INT1_EN_BITS                   0x08    // ICM42670_REG_INT_SOURCE1<3>
#define ICM42670_SMD_INT1_EN_SHIFT                  3       // ICM42670_REG_INT_SOURCE1<3>
#define ICM42670_WOM_Z_INT1_EN_BITS                 0x04    // ICM42670_REG_INT_SOURCE1<2>
#define ICM42670_WOM_Z_INT1_EN_SHIFT                2       // ICM42670_REG_INT_SOURCE1<2>
#define ICM42670_WOM_Y_INT1_EN_BITS                 0x02    // ICM42670_REG_INT_SOURCE1<1>
#define ICM42670_WOM_Y_INT1_EN_SHIFT                1       // ICM42670_REG_INT_SOURCE1<1>
#define ICM42670_WOM_X_INT1_EN_BITS                 0x01    // ICM42670_REG_INT_SOURCE1<0>
#define ICM42670_WOM_X_INT1_EN_SHIFT                0       // ICM42670_REG_INT_SOURCE1<0>

// ICM42670_REG_INT_SOURCE3 and ICM42670_REG_INT_SOURCE4 same as 0 and 1

#define ICM42670_DMP_IDLE_BITS                      0x04    // ICM42670_REG_APEX_DATA3<2>
#define ICM42670_DMP_IDLE_SHIFT                     2       // ICM42670_REG_APEX_DATA3<2>
#define ICM42670_ACTIVITY_CLASS_BITS                0x03    // ICM42670_REG_APEX_DATA3<1:0>
#define ICM42670_ACTIVITY_CLASS_SHIFT               0       // ICM42670_REG_APEX_DATA3<1:0>

#define ICM42670_FIFO_COUNT_FORMAT_BITS             0x40    // ICM42670_REG_INTF_CONFIG0<6>
#define ICM42670_FIFO_COUNT_FORMAT_SHIFT            6       // ICM42670_REG_INTF_CONFIG0<6>
#define ICM42670_FIFO_COUNT_ENDIAN_BITS             0x20    // ICM42670_REG_INTF_CONFIG0<5>
#define ICM42670_FIFO_COUNT_ENDIAN_SHIFT            5       // ICM42670_REG_INTF_CONFIG0<5>
#define ICM42670_SENSOR_DATA_ENDIAN_BITS            0x10    // ICM42670_REG_INTF_CONFIG0<4>
#define ICM42670_SENSOR_DATA_ENDIAN_SHIFT           4       // ICM42670_REG_INTF_CONFIG0<4>

#define ICM42670_I3C_SDR_EN_BITS                    0x08    // ICM42670_REG_INTF_CONFIG1<3>
#define ICM42670_I3C_SDR_EN_SHIFT                   3       // ICM42670_REG_INTF_CONFIG1<3>
#define ICM42670_I3C_DDR_EN_BITS                    0x04    // ICM42670_REG_INTF_CONFIG1<2>
#define ICM42670_I3C_DDR_EN_SHIFT                   2       // ICM42670_REG_INTF_CONFIG1<2>
#define ICM42670_CLKSEL_BITS                        0x03    // ICM42670_REG_INTF_CONFIG1<1:0>
#define ICM42670_CLKSEL_SHIFT                       0       // ICM42670_REG_INTF_CONFIG1<1:0>

#define ICM42670_DATA_RDY_INT_BITS                  0x01    // ICM42670_REG_INT_STATUS_DRDY<0>
#define ICM42670_DATA_RDY_INT_SHIFT                 0       // ICM42670_REG_INT_STATUS_DRDY<0>

#define ICM42670_ST_INT_BITS                        0x80    // ICM42670_REG_INT_STATUS<7>
#define ICM42670_ST_INT_SHIFT                       7       // ICM42670_REG_INT_STATUS<7>
#define ICM42670_FSYNC_INT_BITS                     0x40    // ICM42670_REG_INT_STATUS<6>
#define ICM42670_FSYNC_INT_SHIFT                    6       // ICM42670_REG_INT_STATUS<6>
#define ICM42670_PLL_RDY_INT_BITS                   0x20    // ICM42670_REG_INT_STATUS<5>
#define ICM42670_PLL_RDY_INT_SHIFT                  5       // ICM42670_REG_INT_STATUS<5>
#define ICM42670_RESET_DONE_INT_BITS                0x10    // ICM42670_REG_INT_STATUS<4>
#define ICM42670_RESET_DONE_INT_SHIFT               4       // ICM42670_REG_INT_STATUS<4>
#define ICM42670_FIFO_THS_INT_BITS                  0x04    // ICM42670_REG_INT_STATUS<2>
#define ICM42670_FIFO_THS_INT_SHIFT                 2       // ICM42670_REG_INT_STATUS<2>
#define ICM42670_FIFO_FULL_INT_BITS                 0x02    // ICM42670_REG_INT_STATUS<1>
#define ICM42670_FIFO_FULL_INT_SHIFT                1       // ICM42670_REG_INT_STATUS<1>
#define ICM42670_AGC_RDY_INT_BITS                   0x01    // ICM42670_REG_INT_STATUS<0>
#define ICM42670_AGC_RDY_INT_SHIFT                  0       // ICM42670_REG_INT_STATUS<0>

#define ICM42670_SMD_INT_BITS                       0x08    // ICM42670_REG_INT_STATUS2<3>
#define ICM42670_SMD_INT_SHIFT                      3       // ICM42670_REG_INT_STATUS2<3>
#define ICM42670_WOM_X_INT_BITS                     0x04    // ICM42670_REG_INT_STATUS2<2>
#define ICM42670_WOM_X_INT_SHIFT                    2       // ICM42670_REG_INT_STATUS2<2>
#define ICM42670_WOM_Y_INT_BITS                     0x02    // ICM42670_REG_INT_STATUS2<1>
#define ICM42670_WOM_Y_INT_SHIFT                    1       // ICM42670_REG_INT_STATUS2<1>
#define ICM42670_WOM_Z_INT_BITS                     0x01    // ICM42670_REG_INT_STATUS2<0>
#define ICM42670_WOM_Z_INT_SHIFT                    0       // ICM42670_REG_INT_STATUS2<0>

#define ICM42670_STEP_DET_INT_BITS                  0x20    // ICM42670_REG_INT_STATUS3<5>
#define ICM42670_STEP_DET_INT_SHIFT                 5       // ICM42670_REG_INT_STATUS3<5>
#define ICM42670_STEP_CNT_OVF_INT_BITS              0x10    // ICM42670_REG_INT_STATUS3<4>
#define ICM42670_STEP_CNT_OVF_INT_SHIFT             4       // ICM42670_REG_INT_STATUS3<4>
#define ICM42670_TILT_DET_INT_BITS                  0x08    // ICM42670_REG_INT_STATUS3<3>
#define ICM42670_TILT_DET_INT_SHIFT                 3       // ICM42670_REG_INT_STATUS3<3>
#define ICM42670_FF_DET_INT_BITS                    0x04    // ICM42670_REG_INT_STATUS3<2>
#define ICM42670_FF_DET_INT_SHIFT                   2       // ICM42670_REG_INT_STATUS3<2>
#define ICM42670_LOWG_DET_INT_BITS                  0x02    // ICM42670_REG_INT_STATUS3<1>
#define ICM42670_LOWG_DET_INT_SHIFT                 1       // ICM42670_REG_INT_STATUS3<1>


#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

static inline esp_err_t write_register(icm42670_t *dev, uint8_t reg, uint8_t value)
{
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &value, 1);
}

static inline esp_err_t read_register(icm42670_t *dev, uint8_t reg, uint8_t *value)
{
    return i2c_dev_read_reg(&dev->i2c_dev, reg, value, 1);
}

static inline esp_err_t read_register_16(icm42670_t *dev, uint8_t upper_byte_reg, uint16_t *value)
{
    esp_err_t err;
    uint8_t reg_0, reg_1;
    err = read_register(dev, upper_byte_reg, &reg_1);
    err = read_register(dev, upper_byte_reg + 1, &reg_0);
    *value = reg_0 | (reg_1 << 8);

    return err;
}

static inline esp_err_t manipulate_register(icm42670_t *dev, uint8_t reg_addr, uint8_t mask, uint8_t shift, uint8_t value)
{
    uint8_t reg;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, reg_addr, &reg));
    reg = (reg & ~mask) | (value << shift);
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, reg_addr, reg));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static inline esp_err_t read_mreg_register(icm42670_t *dev, icm42670_mreg_number_t mreg_num, uint8_t reg, uint8_t *value)
{
    bool mclk_rdy;
    CHECK(icm42670_get_mclk_rdy(dev, &mclk_rdy));
    if(!mclk_rdy){
        ESP_LOGE(TAG, "MCLK not running, required to access MREG");
        return ESP_ERR_INVALID_RESPONSE;
    }

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_BLK_SEL_R, mreg_num));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_MADDR_R, reg));
    vTaskDelay(pdMS_TO_TICKS(0.01)); //Wait for 10us until MREG write is complete
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, ICM42670_REG_M_R, value));
    vTaskDelay(pdMS_TO_TICKS(0.01)); 
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

static inline esp_err_t write_mreg_register(icm42670_t *dev, icm42670_mreg_number_t mreg_num, uint8_t reg, uint8_t value)
{
    bool mclk_rdy;
    CHECK(icm42670_get_mclk_rdy(dev, &mclk_rdy));
    if(!mclk_rdy){
        ESP_LOGE(TAG, "MCLK not running, required to access MREG");
        return ESP_ERR_INVALID_RESPONSE;
    }

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_BLK_SEL_W, mreg_num));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_MADDR_W, reg));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_M_W, value));
    vTaskDelay(pdMS_TO_TICKS(0.01)); //Wait for 10us until MREG write is complete
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

static inline esp_err_t manipulate_mreg_register(icm42670_t *dev, icm42670_mreg_number_t mreg_num, uint8_t reg_addr, uint8_t mask, uint8_t shift, uint8_t value)
{
    uint8_t reg;
    CHECK(read_mreg_register(dev, mreg_num, reg_addr, &reg));
    reg = (reg & ~mask) | (value << shift);
    CHECK(write_mreg_register(dev, mreg_num, reg_addr, reg));

    return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////

esp_err_t icm42670_init_desc(icm42670_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != ICM42670_I2C_ADDR_GND && addr != ICM42670_I2C_ADDR_VCC)
    {
        ESP_LOGE(TAG, "Invalid I2C address `0x%x`: must be one of 0x%x, 0x%x",
                addr, ICM42670_I2C_ADDR_GND, ICM42670_I2C_ADDR_VCC);
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

esp_err_t icm42670_free_desc(icm42670_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t icm42670_init(icm42670_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t reg;
    
    // check who_am_i register
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, ICM42670_REG_WHO_AM_I, &reg));
    if (reg != 0x67)
    {
        ESP_LOGE(TAG, "Error initializing icm42670, who_am_i register did not return 0x67");
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_ERR_INVALID_RESPONSE;
    }
    // check if internal clock is running
    bool mclk_rdy = false;
    CHECK(icm42670_get_mclk_rdy(dev, &mclk_rdy));
    if(!mclk_rdy)
    {
        ESP_LOGE(TAG, "Error initializing icm42670, Internal clock not running");
        return ESP_ERR_INVALID_RESPONSE;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t icm42670_set_gyro_pwr_mode(icm42670_t *dev, icm42670_gyro_pwr_mode_t pwr_mode)
{
    CHECK_ARG(dev && pwr_mode);

    CHECK(manipulate_register(dev, ICM42670_REG_PWR_MGMT0, ICM42670_GYRO_MODE_BITS, ICM42670_GYRO_MODE_SHIFT, pwr_mode));
    return ESP_OK;
}

esp_err_t icm42670_set_accel_pwr_mode(icm42670_t *dev, icm42670_accel_pwr_mode_t pwr_mode)
{
    CHECK_ARG(dev && pwr_mode);

    CHECK(manipulate_register(dev, ICM42670_REG_PWR_MGMT0, ICM42670_ACCEL_MODE_BITS, ICM42670_ACCEL_MODE_SHIFT, pwr_mode));
    return ESP_OK;
}

esp_err_t icm42670_read_raw_data(icm42670_t *dev, uint8_t data_register, uint16_t *data)
{
    CHECK_ARG(dev && data_register && data);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register_16(dev, data_register, data));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t icm42670_read_temperature(icm42670_t *dev, float *temperature)
{
    CHECK_ARG(dev && temperature);

    uint16_t reg;
    CHECK(icm42670_read_raw_data(dev, ICM42670_REG_TEMP_DATA1, &reg));
    *temperature = (reg / 128.0) + 25;
    return ESP_OK;
}

esp_err_t icm42670_reset(icm42670_t *dev)
{
    CHECK_ARG(dev);

    uint8_t reg = 1 << ICM42670_SOFT_RESET_DEVICE_CONFIG_SHIFT;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_SIGNAL_PATH_RESET, reg));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t icm42670_flush_fifo(icm42670_t *dev)
{
    CHECK_ARG(dev);

    uint8_t reg = 1 << ICM42670_FIFO_FLUSH_SHIFT;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_SIGNAL_PATH_RESET, reg));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    vTaskDelay(pdMS_TO_TICKS(0.002)); //flush is done within 1.5us

    return ESP_OK;
}

esp_err_t icm42670_set_gyro_fsr(icm42670_t *dev, icm42670_gyro_fsr_t range)
{
    CHECK_ARG(dev && range);
    return manipulate_register(dev, ICM42670_REG_GYRO_CONFIG0, ICM42670_GYRO_UI_FS_SEL_BITS, ICM42670_GYRO_UI_FS_SEL_SHIFT, range);
}

esp_err_t icm42670_set_gyro_odr(icm42670_t *dev, icm42670_gyro_odr_t odr)
{
    CHECK_ARG(dev && odr);
    return manipulate_register(dev, ICM42670_REG_GYRO_CONFIG0, ICM42670_GYRO_ODR_BITS, ICM42670_GYRO_ODR_SHIFT, odr);
}

esp_err_t icm42670_set_accel_fsr(icm42670_t *dev, icm42670_accel_fsr_t range)
{
    CHECK_ARG(dev && range);
    return manipulate_register(dev, ICM42670_REG_ACCEL_CONFIG0, ICM42670_ACCEL_UI_FS_SEL_BITS, ICM42670_ACCEL_UI_FS_SEL_SHIFT, range);
}

esp_err_t icm42670_set_accel_odr(icm42670_t *dev, icm42670_accel_odr_t odr)
{
    CHECK_ARG(dev && odr);
    return manipulate_register(dev, ICM42670_REG_ACCEL_CONFIG0, ICM42670_ACCEL_ODR_BITS, ICM42670_ACCEL_ODR_SHIFT, odr);
}

esp_err_t icm42670_set_temp_lpf(icm42670_t *dev, icm42670_temp_lfp_t lpf_bw)
{
    CHECK_ARG(dev && lpf_bw);
    return manipulate_register(dev, ICM42670_REG_TEMP_CONFIG0, ICM42670_TEMP_FILT_BW_BITS, ICM42670_TEMP_FILT_BW_SHIFT, lpf_bw);
}

esp_err_t icm42670_set_gyro_lpf(icm42670_t *dev, icm42670_gyro_lfp_t lpf_bw)
{
    CHECK_ARG(dev && lpf_bw);
    return manipulate_register(dev, ICM42670_REG_GYRO_CONFIG1, ICM42670_GYRO_UI_FILT_BW_BITS, ICM42670_GYRO_UI_FILT_BW_SHIFT, lpf_bw);
}

esp_err_t icm42670_set_accel_lpf(icm42670_t *dev, icm42670_accel_lfp_t lpf_bw)
{
    CHECK_ARG(dev && lpf_bw);
    return manipulate_register(dev, ICM42670_REG_ACCEL_CONFIG1, ICM42670_ACCEL_UI_FILT_BW_BITS, ICM42670_ACCEL_UI_FILT_BW_SHIFT, lpf_bw);
}

esp_err_t icm42670_set_accel_avg(icm42670_t *dev, icm42670_accel_avg_t avg)
{
    CHECK_ARG(dev && avg);
    return manipulate_register(dev, ICM42670_REG_ACCEL_CONFIG1, ICM42670_ACCEL_UI_AVG_BITS, ICM42670_ACCEL_UI_AVG_SHIFT, avg);
}

esp_err_t icm42670_config_int_pin(icm42670_t *dev, uint8_t int_pin, icm42670_int_config_t config)
{
    CHECK_ARG(dev && int_pin);

    uint8_t reg = config.mode << 2 | config.drive << 1 | config.polarity;
    if(int_pin == 2){
        return manipulate_register(dev, ICM42670_REG_INT_CONFIG, 0b00111000, ICM42670_INT2_POLARITY_SHIFT, reg);
    }else if (int_pin == 1){
        return manipulate_register(dev, ICM42670_REG_INT_CONFIG, 0b00000111, ICM42670_INT1_POLARITY_SHIFT, reg);
    }else{
        printf("Error, only INT pins 1 and 2 available\n");
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t icm42670_set_int_sources(icm42670_t *dev, uint8_t int_pin, icm42670_int_source_t sources)
{
    CHECK_ARG(dev && int_pin);

    uint8_t reg1 = 0, reg2 = 0;
    if(sources.self_test_done)
        reg1 = reg1 | (1 << ICM42670_ST_INT1_EN_SHIFT);
    if(sources.fsync)
        reg1 = reg1 | (1 << ICM42670_FSYNC_INT1_EN_SHIFT);
    if(sources.pll_ready)
        reg1 = reg1 | (1 << ICM42670_PLL_RDY_INT1_EN_SHIFT);
    if(sources.reset_done)
        reg1 = reg1 | (1 << ICM42670_RESET_DONE_INT1_EN_SHIFT);
    if(sources.data_ready)
        reg1 = reg1 | (1 << ICM42670_DRDY_INT1_EN_SHIFT);
    if(sources.fifo_threshold)
        reg1 = reg1 | (1 << ICM42670_FIFO_THS_INT1_EN_SHIFT);
    if(sources.fifo_full)
        reg1 = reg1 | (1 << ICM42670_FIFO_FULL_INT1_EN_SHIFT);
    if(sources.agc_ready)
        reg1 = reg1 | (1 << ICM42670_AGC_RDY_INT1_EN_SHIFT);
    if(sources.i3c_error)
        reg2 = reg2 | (1 << ICM42670_I3C_PROTOCOL_ERROR_INT1_EN_SHIFT);
    if(sources.smd)
        reg2 = reg2 | (1 << ICM42670_SMD_INT1_EN_SHIFT);
    if(sources.wom_z)
        reg2 = reg2 | (1 << ICM42670_WOM_Z_INT1_EN_SHIFT);
    if(sources.wom_y)
        reg2 = reg2 | (1 << ICM42670_WOM_Y_INT1_EN_SHIFT);
    if(sources.wom_x)
        reg2 = reg2 | (1 << ICM42670_WOM_X_INT1_EN_SHIFT);

    if(int_pin == 1){
        I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
        I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_INT_SOURCE0, reg1));
        I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_INT_SOURCE1, reg2));
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    }else if (int_pin == 2){
        I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
        I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_INT_SOURCE3, reg1));
        I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, ICM42670_REG_INT_SOURCE4, reg2));
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    }else{
        printf("Error, only INT pins 1 and 2 available\n");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t icm42670_config_wom(icm42670_t *dev, icm42670_wom_config_t config)
{
    CHECK_ARG(dev);

    CHECK(manipulate_register(dev, ICM42670_REG_WOM_CONFIG, ICM42670_WOM_INT_DUR_BITS, ICM42670_WOM_INT_DUR_SHIFT, config.trigger));
    CHECK(manipulate_register(dev, ICM42670_REG_WOM_CONFIG, ICM42670_WOM_INT_MODE_BITS, ICM42670_WOM_INT_MODE_SHIFT, config.logical_mode));
    CHECK(manipulate_register(dev, ICM42670_REG_WOM_CONFIG, ICM42670_WOM_MODE_BITS, ICM42670_WOM_MODE_SHIFT, config.reference));

    // WoM threshold values
    CHECK(write_mreg_register(dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_X_THR, config.wom_x_threshold));
    CHECK(write_mreg_register(dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_Y_THR, config.wom_y_threshold));
    CHECK(write_mreg_register(dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_Z_THR, config.wom_z_threshold));

    return ESP_OK;
}

esp_err_t icm42670_enable_wom(icm42670_t *dev, bool enable)
{
    CHECK_ARG(dev && enable);
    return manipulate_register(dev, ICM42670_REG_WOM_CONFIG, ICM42670_WOM_EN_BITS, ICM42670_WOM_EN_SHIFT, enable);
}

esp_err_t icm42670_get_mclk_rdy(icm42670_t *dev, bool *mclk_rdy)
{
    CHECK_ARG(dev && mclk_rdy);

    uint8_t reg;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, ICM42670_REG_MCLK_RDY, &reg));
    if ((reg & ICM42670_MCLK_RDY_BITS) >> ICM42670_MCLK_RDY_SHIFT)
        *mclk_rdy = true;
    else
        *mclk_rdy = false;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}
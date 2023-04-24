
/*
 * The MIT License (MIT)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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
 *
 */

/**
 * @file mpu6050.c
 *  Thanks to @see https://github.com/gabrielbvicari/esp32-mpu6050 for driver.
 *  Improved by Raghav Jha https://github.com/horsemann07
 */

/* C headers */
#include <math.h>

/* esp headers */
#include <esp_log.h>

/* mpu6050 headers */
#include "mpu6050.h"

#define TAG (__FILENAME__)

// Internal Registers Definitions
#define MPU6050_REGISTER_XG_OFFS_TC         (0)
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

// DLPF values
#define MPU6050_DLPF_BW_256 (0x00)
#define MPU6050_DLPF_BW_188 (0x01)
#define MPU6050_DLPF_BW_98  (0x02)
#define MPU6050_DLPF_BW_42  (0x03)
#define MPU6050_DLPF_BW_20  (0x04)
#define MPU6050_DLPF_BW_10  (0x05)
#define MPU6050_DLPF_BW_5   (0x06)

// DHPF values:
#define MPU6050_DHPF_RESET (0x00)
#define MPU6050_DHPF_5     (0x01)
#define MPU6050_DHPF_2P5   (0x02)
#define MPU6050_DHPF_1P25  (0x03)
#define MPU6050_DHPF_0P63  (0x04)
#define MPU6050_DHPF_HOLD  (0x07)

// Full scale gyroscope range:
#define MPU6050_GYRO_FULL_SCALE_RANGE_250  (0x00)
#define MPU6050_GYRO_FULL_SCALE_RANGE_500  (0x01)
#define MPU6050_GYRO_FULL_SCALE_RANGE_1000 (0x02)
#define MPU6050_GYRO_FULL_SCALE_RANGE_2000 (0x03)

// Full scale accelerometer range:
#define MPU6050_ACCEL_FULL_SCALE_RANGE_2  (0x00)
#define MPU6050_ACCEL_FULL_SCALE_RANGE_4  (0x01)
#define MPU6050_ACCEL_FULL_SCALE_RANGE_8  (0x02)
#define MPU6050_ACCEL_FULL_SCALE_RANGE_16 (0x03)

// Interrupt values:
#define MPU6050_INTMODE_ACTIVEHIGH  (0x00)
#define MPU6050_INTMODE_ACTIVELOW   (0x01)
#define MPU6050_INTDRV_PUSHPULL     (0x00)
#define MPU6050_INTDRV_OPENDRAIN    (0x01)
#define MPU6050_INTLATCH_50USPULSE  (0x00)
#define MPU6050_INTLATCH_WAITCLEAR  (0x01)
#define MPU6050_INTCLEAR_STATUSREAD (0x00)
#define MPU6050_INTCLEAR_ANYREAD    (0x01)

// Clock sources:
#define MPU6050_CLOCK_INTERNAL         (0x00)
#define MPU6050_CLOCK_PLL_XGYRO        (0x01)
#define MPU6050_CLOCK_PLL_YGYRO        (0x02)
#define MPU6050_CLOCK_PLL_ZGYRO        (0x03)
#define MPU6050_CLOCK_PLL_EXTERNAL_32K (0x04)
#define MPU6050_CLOCK_PLL_EXTERNAL_19M (0x05)
#define MPU6050_CLOCK_KEEP_RESET       (0x07)

// Wake frequencies:
#define MPU6050_WAKE_FREQ_1P25 (0x0)
#define MPU6050_WAKE_FREQ_2P5  (0x1)
#define MPU6050_WAKE_FREQ_5    (0x2)
#define MPU6050_WAKE_FREQ_10   (0x3)

// Decrement values:
#define MPU6050_DETECT_DECREMENT_RESET (0x0)
#define MPU6050_DETECT_DECREMENT_1     (0x1)
#define MPU6050_DETECT_DECREMENT_2     (0x2)
#define MPU6050_DETECT_DECREMENT_4     (0x3)

// External sync values:
#define MPU6050_EXT_SYNC_DISABLED     (0x0)
#define MPU6050_EXT_SYNC_TEMP_OUT_L   (0x1)
#define MPU6050_EXT_SYNC_GYRO_XOUT_L  (0x2)
#define MPU6050_EXT_SYNC_GYRO_YOUT_L  (0x3)
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L  (0x4)
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L (0x5)
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L (0x6)
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L (0x7)

// Clock division values:
#define MPU6050_CLOCK_DIV_348 (0x0)
#define MPU6050_CLOCK_DIV_333 (0x1)
#define MPU6050_CLOCK_DIV_320 (0x2)
#define MPU6050_CLOCK_DIV_308 (0x3)
#define MPU6050_CLOCK_DIV_296 (0x4)
#define MPU6050_CLOCK_DIV_286 (0x5)
#define MPU6050_CLOCK_DIV_276 (0x6)
#define MPU6050_CLOCK_DIV_267 (0x7)
#define MPU6050_CLOCK_DIV_258 (0x8)
#define MPU6050_CLOCK_DIV_500 (0x9)
#define MPU6050_CLOCK_DIV_471 (0xA)
#define MPU6050_CLOCK_DIV_444 (0xB)
#define MPU6050_CLOCK_DIV_421 (0xC)
#define MPU6050_CLOCK_DIV_400 (0xD)
#define MPU6050_CLOCK_DIV_381 (0xE)
#define MPU6050_CLOCK_DIV_364 (0xF)

// Bit and length defines for SELF_TEST register:
#define MPU6050_SELF_TEST_XA_1_BIT    (0x07)
#define MPU6050_SELF_TEST_XA_1_LENGTH (0x03)
#define MPU6050_SELF_TEST_XA_2_BIT    (0x05)
#define MPU6050_SELF_TEST_XA_2_LENGTH (0x02)
#define MPU6050_SELF_TEST_YA_1_BIT    (0x07)
#define MPU6050_SELF_TEST_YA_1_LENGTH (0x03)
#define MPU6050_SELF_TEST_YA_2_BIT    (0x03)
#define MPU6050_SELF_TEST_YA_2_LENGTH (0x02)
#define MPU6050_SELF_TEST_ZA_1_BIT    (0x07)
#define MPU6050_SELF_TEST_ZA_1_LENGTH (0x03)
#define MPU6050_SELF_TEST_ZA_2_BIT    (0x01)
#define MPU6050_SELF_TEST_ZA_2_LENGTH (0x02)
#define MPU6050_SELF_TEST_XG_1_BIT    (0x04)
#define MPU6050_SELF_TEST_XG_1_LENGTH (0x05)
#define MPU6050_SELF_TEST_YG_1_BIT    (0x04)
#define MPU6050_SELF_TEST_YG_1_LENGTH (0x05)
#define MPU6050_SELF_TEST_ZG_1_BIT    (0x04)
#define MPU6050_SELF_TEST_ZG_1_LENGTH (0x05)

// Bit and length defines for CONFIG register:
#define MPU6050_CFG_EXT_SYNC_SET_BIT    (5)
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH (3)
#define MPU6050_CFG_DLPF_CFG_BIT        (2)
#define MPU6050_CFG_DLPF_CFG_LENGTH     (3)

// Bit and length defines for GYRO_CONFIG register:
#define MPU6050_GCONFIG_FS_SEL_BIT    (4)
#define MPU6050_GCONFIG_FS_SEL_LENGTH (2)

// Bit and length defines for ACCEL_CONFIG register:
#define MPU6050_ACONFIG_XA_ST_BIT        (7)
#define MPU6050_ACONFIG_YA_ST_BIT        (6)
#define MPU6050_ACONFIG_ZA_ST_BIT        (5)
#define MPU6050_ACONFIG_AFS_SEL_BIT      (4)
#define MPU6050_ACONFIG_AFS_SEL_LENGTH   (2)
#define MPU6050_ACONFIG_ACCEL_HPF_BIT    (2)
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH (3)

// Bit and length defines for FIFO_EN register:
#define MPU6050_TEMP_FIFO_EN_BIT  (7)
#define MPU6050_XG_FIFO_EN_BIT    (6)
#define MPU6050_YG_FIFO_EN_BIT    (5)
#define MPU6050_ZG_FIFO_EN_BIT    (4)
#define MPU6050_ACCEL_FIFO_EN_BIT (3)
#define MPU6050_SLV2_FIFO_EN_BIT  (2)
#define MPU6050_SLV1_FIFO_EN_BIT  (1)
#define MPU6050_SLV0_FIFO_EN_BIT  (0)

// Bit and length defines for I2C_MST_CTRL register:
#define MPU6050_MULT_MST_EN_BIT    (7)
#define MPU6050_WAIT_FOR_ES_BIT    (6)
#define MPU6050_SLV_3_FIFO_EN_BIT  (5)
#define MPU6050_I2C_MST_P_NSR_BIT  (4)
#define MPU6050_I2C_MST_CLK_BIT    (3)
#define MPU6050_I2C_MST_CLK_LENGTH (4)

// Bit and length defines for I2C_SLV* register:
#define MPU6050_I2C_SLV_RW_BIT      (7)
#define MPU6050_I2C_SLV_ADDR_BIT    (6)
#define MPU6050_I2C_SLV_ADDR_LENGTH (7)
#define MPU6050_I2C_SLV_EN_BIT      (7)
#define MPU6050_I2C_SLV_BYTE_SW_BIT (6)
#define MPU6050_I2C_SLV_REG_DIS_BIT (5)
#define MPU6050_I2C_SLV_GRP_BIT     (4)
#define MPU6050_I2C_SLV_LEN_BIT     (3)
#define MPU6050_I2C_SLV_LEN_LENGTH  (4)

// Bit and length defines for I2C_SLV4 register:
#define MPU6050_I2C_SLV4_RW_BIT         (7)
#define MPU6050_I2C_SLV4_ADDR_BIT       (6)
#define MPU6050_I2C_SLV4_ADDR_LENGTH    (7)
#define MPU6050_I2C_SLV4_EN_BIT         (7)
#define MPU6050_I2C_SLV4_INT_EN_BIT     (6)
#define MPU6050_I2C_SLV4_REG_DIS_BIT    (5)
#define MPU6050_I2C_SLV4_MST_DLY_BIT    (4)
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH (5)

// Bit and length defines for I2C_MST_STATUS register:
#define MPU6050_MST_PASS_THROUGH_BIT  (7)
#define MPU6050_MST_I2C_SLV4_DONE_BIT (6)
#define MPU6050_MST_I2C_LOST_ARB_BIT  (5)
#define MPU6050_MST_I2C_SLV4_NACK_BIT (4)
#define MPU6050_MST_I2C_SLV3_NACK_BIT (3)
#define MPU6050_MST_I2C_SLV2_NACK_BIT (2)
#define MPU6050_MST_I2C_SLV1_NACK_BIT (1)
#define MPU6050_MST_I2C_SLV0_NACK_BIT (0)

// Bit and length defines for INT_PIN_CFG register:
#define MPU6050_INTCFG_INT_LEVEL_BIT       (7)
#define MPU6050_INTCFG_INT_OPEN_BIT        (6)
#define MPU6050_INTCFG_LATCH_INT_EN_BIT    (5)
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT    (4)
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT (3)
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT    (2)
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT   (1)
#define MPU6050_INTCFG_CLKOUT_EN_BIT       (0)

// Bit and length defines for INT_ENABLE and INT_STATUS registers:
#define MPU6050_INTERRUPT_FF_BIT          (7)
#define MPU6050_INTERRUPT_MOT_BIT         (6)
#define MPU6050_INTERRUPT_ZMOT_BIT        (5)
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT  (4)
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT (3)
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT (2)
#define MPU6050_INTERRUPT_DMP_INT_BIT     (1)
#define MPU6050_INTERRUPT_DATA_RDY_BIT    (0)

// Bit and length defines for MOT_DETECT_STATUS register:
#define MPU6050_MOTION_MOT_XNEG_BIT  (7)
#define MPU6050_MOTION_MOT_XPOS_BIT  (6)
#define MPU6050_MOTION_MOT_YNEG_BIT  (5)
#define MPU6050_MOTION_MOT_YPOS_BIT  (4)
#define MPU6050_MOTION_MOT_ZNEG_BIT  (3)
#define MPU6050_MOTION_MOT_ZPOS_BIT  (2)
#define MPU6050_MOTION_MOT_ZRMOT_BIT (0)

// Bit and length defines for I2C_MST_DELAY_CTRL register:
#define MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT (7)
#define MPU6050_DLYCTRL_I2C_SLV4_DLY_EN_BIT (4)
#define MPU6050_DLYCTRL_I2C_SLV3_DLY_EN_BIT (3)
#define MPU6050_DLYCTRL_I2C_SLV2_DLY_EN_BIT (2)
#define MPU6050_DLYCTRL_I2C_SLV1_DLY_EN_BIT (1)
#define MPU6050_DLYCTRL_I2C_SLV0_DLY_EN_BIT (0)

// Bit and length defines for SIGNAL_PATH_RESET register:
#define MPU6050_PATHRESET_GYRO_RESET_BIT  (2)
#define MPU6050_PATHRESET_ACCEL_RESET_BIT (1)
#define MPU6050_PATHRESET_TEMP_RESET_BIT  (0)

// Bit and length defines for MOT_DETECT_CTRL register:
#define MPU6050_DETECT_ACCEL_DELAY_BIT    (5)
#define MPU6050_DETECT_ACCEL_DELAY_LENGTH (2)
#define MPU6050_DETECT_FF_COUNT_BIT       (3)
#define MPU6050_DETECT_FF_COUNT_LENGTH    (2)
#define MPU6050_DETECT_MOT_COUNT_BIT      (1)
#define MPU6050_DETECT_MOT_COUNT_LENGTH   (2)

// Bit and length defines for USER_CTRL register:
#define MPU6050_USERCTRL_DMP_EN_BIT         (7)
#define MPU6050_USERCTRL_FIFO_EN_BIT        (6)
#define MPU6050_USERCTRL_I2C_MST_EN_BIT     (5)
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT     (4)
#define MPU6050_USERCTRL_DMP_RESET_BIT      (3)
#define MPU6050_USERCTRL_FIFO_RESET_BIT     (2)
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT  (1)
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT (0)

// Bit and length defines for PWR_MGMT_1 register:
#define MPU6050_PWR1_DEVICE_RESET_BIT (7)
#define MPU6050_PWR1_SLEEP_BIT        (6)
#define MPU6050_PWR1_CYCLE_BIT        (5)
#define MPU6050_PWR1_TEMP_DIS_BIT     (3)
#define MPU6050_PWR1_CLKSEL_BIT       (2)
#define MPU6050_PWR1_CLKSEL_LENGTH    (3)

// Bit and length defines for PWR_MGMT_2 register:
#define MPU6050_PWR2_LP_WAKE_CTRL_BIT    (7)
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH (2)
#define MPU6050_PWR2_STBY_XA_BIT         (5)
#define MPU6050_PWR2_STBY_YA_BIT         (4)
#define MPU6050_PWR2_STBY_ZA_BIT         (3)
#define MPU6050_PWR2_STBY_XG_BIT         (2)
#define MPU6050_PWR2_STBY_YG_BIT         (1)
#define MPU6050_PWR2_STBY_ZG_BIT         (0)

// Bit and length defines for WHO_AM_I register:
#define MPU6050_WHO_AM_I_BIT    (6)
#define MPU6050_WHO_AM_I_LENGTH (6)

// Undocumented bits and lengths:
#define MPU6050_TC_PWR_MODE_BIT    (7)
#define MPU6050_TC_OFFSET_BIT      (6)
#define MPU6050_TC_OFFSET_LENGTH   (6)
#define MPU6050_TC_OTP_BNK_VLD_BIT (0)
#define MPU6050_DMPINT_5_BIT       (5)
#define MPU6050_DMPINT_4_BIT       (4)
#define MPU6050_DMPINT_3_BIT       (3)
#define MPU6050_DMPINT_2_BIT       (2)
#define MPU6050_DMPINT_1_BIT       (1)
#define MPU6050_DMPINT_0_BIT       (0)

#define PI              (3.14159265358979323846f)
#define GYRO_MEAS_ERROR (PI * (60.0f / 180.0f))
#define GYRO_MEAS_DRIFT (PI * (1.0f / 180.0f))
#define BETA            (sqrt(3.0f / 4.0f) * GYRO_MEAS_ERROR)
#define ZETA            (sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT)

#define ENABLE  (ESP_OK)
#define DISABLE (ESP_FAIL)

// Max 1MHz for esp-idf, but device supports up to 1.7Mhz
#define I2C_FREQ_HZ (1000000)

/*
 * Macro which can be used to check parameters.
 * Prints the error code, error location, and the failed statement to serial output.
 */
#define ESP_PARAM_CHECK(con)                                                                                           \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(con))                                                                                                    \
        {                                                                                                              \
            ESP_LOGE(TAG, "[%s, %d]: <ESP_ERR_INVALID_ARG> !(%s)", __func__, __LINE__, #con);                          \
            return ESP_ERR_INVALID_ARG;                                                                                \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

/*
 * Macro which can be used to check the error code,
 * and terminate the program in case the code is not ESP_OK.
 */
#define ESP_ERROR_RETURN(ret)                                                                                          \
    do                                                                                                                 \
    {                                                                                                                  \
        esp_err_t __ = ret;                                                                                            \
        if (__ != ESP_OK)                                                                                              \
        {                                                                                                              \
            return __;                                                                                                 \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

/**
 * Macro which can be used to check the error code,
 * and terminate the program in case the code is not ESP_OK.
 * In debug mode, it prints the error code, error location to serial output.
 */
#define ESP_ERROR_CDEBUG(ret)                                                                                          \
    if (ret != ESP_OK)                                                                                                 \
    {                                                                                                                  \
        ESP_LOGD(TAG, "[%s, %d] <%s> ", __func__, __LINE__, esp_err_to_name(ret));                                     \
        return ret;                                                                                                    \
    }

static float quart[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static float delta_t = 0.0f;

/* ----------------------------------------------------- */
static esp_err_t i2c_read_bytes(i2c_dev_t *dev, uint8_t reg_addr, size_t size, uint8_t *data)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, &(reg_addr), sizeof(uint8_t), data, size));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
/* ----------------------------------------------------- */
static esp_err_t i2c_read_byte(i2c_dev_t *dev, uint8_t reg_addr, uint8_t *data)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg_addr, (void *)data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
/* ----------------------------------------------------- */
static esp_err_t i2c_read_bits(i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_start, uint8_t size, uint8_t *data)
{
    I2C_DEV_TAKE_MUTEX(dev);

    uint8_t bit;
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg_addr, (void *)&(bit), 1));

    uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);
    bit &= mask;
    bit >>= (bit_start - size + 1);
    *data = bit;
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
/* ----------------------------------------------------- */
static esp_err_t i2c_read_bit(i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_number, uint8_t *data)
{
    I2C_DEV_TAKE_MUTEX(dev);

    uint8_t bit;
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg_addr, (void *)&(bit), 1));
    *data = bit & (1 << bit_number);
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
/* ----------------------------------------------------- */
// static esp_err_t i2c_write_bytes(i2c_dev_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t size)
// {
//     I2C_DEV_TAKE_MUTEX(dev);
//     I2C_DEV_CHECK(dev, i2c_dev_write(dev, &(reg_addr), sizeof(reg_addr), data, size));
//     I2C_DEV_GIVE_MUTEX(dev);
//     return ESP_OK;
// }
/* ----------------------------------------------------- */
static esp_err_t i2c_write_byte(i2c_dev_t *dev, uint8_t reg_addr, uint8_t data)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg_addr, &(data), 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
/* ----------------------------------------------------- */
static esp_err_t i2c_write_bits(i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_start, uint8_t size, uint8_t data)
{
    I2C_DEV_TAKE_MUTEX(dev);
    uint8_t bit = 0;
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg_addr, (void *)&(bit), 1));

    uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);
    data <<= (bit_start - size + 1);
    data &= mask;
    bit &= ~(mask);
    bit |= data;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg_addr, &(bit), 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
/* ----------------------------------------------------- */
static esp_err_t i2c_write_bit(i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_number, uint8_t data)
{
    I2C_DEV_TAKE_MUTEX(dev);
    uint8_t bit;

    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg_addr, (void *)&(bit), 1));

    if (data != 0)
    {
        bit = (bit | (1 << bit_number));
    }
    else
    {
        bit = (bit & ~(1 << bit_number));
    }
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg_addr, &(bit), 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
/* ----------------------------------------------------- */
static esp_err_t i2c_write_word(i2c_dev_t *dev, uint8_t reg_addr, uint8_t data)
{
    I2C_DEV_TAKE_MUTEX(dev);
    uint8_t data_1[] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &(reg_addr), sizeof(reg_addr), (uint8_t *)&(data_1), 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

/**********************************************************/

/* ------------------------------------------------------ */
esp_err_t mpu6050_get_aux_vddio_level(mpu6050_dev_t *setting, uint8_t *level)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, level));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_aux_vddio_level(mpu6050_dev_t *setting, uint8_t level)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, level));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_rate(mpu6050_dev_t *setting, uint8_t *rate)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_SMPLRT_DIV, rate));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_rate(mpu6050_dev_t *setting, uint8_t rate)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_SMPLRT_DIV, rate));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_external_frame_sync(mpu6050_dev_t *setting, uint8_t *sync)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
        MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_external_frame_sync(mpu6050_dev_t *setting, uint8_t sync)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
        MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dlpf_mode(mpu6050_dev_t *setting, uint8_t *mode)
{
    return (
        i2c_read_bits(setting, MPU6050_REGISTER_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_dlpf_mode(mpu6050_dev_t *setting, uint8_t mode)
{
    return (
        i2c_write_bits(setting, MPU6050_REGISTER_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_full_scale_gyro_range(mpu6050_dev_t *setting, uint8_t *gyro_range)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
        MPU6050_GCONFIG_FS_SEL_LENGTH, gyro_range));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_full_scale_gyro_range(mpu6050_dev_t *setting, uint8_t range)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
        MPU6050_GCONFIG_FS_SEL_LENGTH, range));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_x_self_test_factory_trim(mpu6050_dev_t *setting, uint8_t *accel_x)
{
    uint8_t a = 0;
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_X, accel_x));
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_A, &a));
    *accel_x = ((*accel_x) >> 3) | ((a >> 4) & 0x03);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_y_self_test_factory_trim(mpu6050_dev_t *setting, uint8_t *accel_y)
{
    uint8_t a = 0;
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_Y, accel_y));
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_A, &a));
    *accel_y = ((*accel_y) >> 3) | ((a >> 2) & 0x03);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_z_self_test_factory_trim(mpu6050_dev_t *setting, uint8_t *accel_z)
{
    uint8_t a = 0;
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_Z, accel_z));
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_A, &a));
    *accel_z = ((*accel_z) >> 3) | (a & 0x03);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_gyro_x_self_test_factory_trim(mpu6050_dev_t *setting, uint8_t *gypr_x)
{
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_X, gypr_x));
    *gypr_x = ((*gypr_x) & 0x1F);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_gyro_y_self_test_factory_trim(mpu6050_dev_t *setting, uint8_t *gypr_y)
{
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_Y, gypr_y));
    *gypr_y = (*gypr_y & 0x1F);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_gyro_z_self_test_factory_trim(mpu6050_dev_t *setting, uint8_t *gypr_z)
{
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_SELF_TEST_Z, gypr_z));
    *gypr_z = (*gypr_z & 0x1F);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_x_self_test(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, &(byte)));
    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_accel_x_self_test(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_y_self_test(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_accel_y_self_test(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_z_self_test(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_accel_z_self_test(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_full_scale_accel_range(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bits(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
        MPU6050_ACONFIG_AFS_SEL_LENGTH, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_full_scale_accel_range(mpu6050_dev_t *setting, uint8_t range)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
        MPU6050_ACONFIG_AFS_SEL_LENGTH, range));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dhpf_mode(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bits(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT,
        MPU6050_ACONFIG_ACCEL_HPF_LENGTH, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_dhpf_mode(mpu6050_dev_t *setting, uint8_t mode)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT,
        MPU6050_ACONFIG_ACCEL_HPF_LENGTH, mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_freefall_detection_threshold(mpu6050_dev_t *setting, uint8_t *threshold)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_FF_THR, threshold));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_freefall_detection_threshold(mpu6050_dev_t *setting, uint8_t threshold)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_FF_THR, threshold));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_freefall_detection_duration(mpu6050_dev_t *setting, uint8_t *duration)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_FF_DUR, duration));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_freefall_detection_duration(mpu6050_dev_t *setting, uint8_t duration)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_FF_DUR, duration));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_motion_detection_threshold(mpu6050_dev_t *setting, uint8_t *threshold)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_MOT_THR, threshold));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_motion_detection_threshold(mpu6050_dev_t *setting, uint8_t threshold)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_MOT_THR, threshold));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_motion_detection_duration(mpu6050_dev_t *setting, uint8_t *duration)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_MOT_DUR, duration));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_motion_detection_duration(mpu6050_dev_t *setting, uint8_t duration)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_MOT_DUR, duration));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_zero_motion_detection_threshold(mpu6050_dev_t *setting, uint8_t *threshold)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_ZRMOT_THR, threshold));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_zero_motion_detection_threshold(mpu6050_dev_t *setting, uint8_t threshold)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_ZRMOT_THR, threshold));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_zero_motion_detection_duration(mpu6050_dev_t *setting, uint8_t *duration)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_ZRMOT_DUR, duration));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_zero_motion_detection_duration(mpu6050_dev_t *setting, uint8_t duration)
{
    return i2c_write_byte(setting, MPU6050_REGISTER_ZRMOT_DUR, duration);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_temp_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_temp_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_gyro_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_x_gyro_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_gyro_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_y_gyro_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_gyro_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_z_gyro_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accel_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_accel_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_2_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_2_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_1_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_1_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_0_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_0_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_multi_master_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_multi_master_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_wait_for_external_sensor_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_wait_for_external_sensor_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_3_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_3_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_read_write_transition_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_read_write_transition_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_master_clock_speed(mpu6050_dev_t *setting, uint8_t *clk_spd)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH,
        clk_spd));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_master_clock_speed(mpu6050_dev_t *setting, uint8_t clk_spd)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH,
        clk_spd));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_address(mpu6050_dev_t *setting, uint8_t num, uint8_t *addr)
{
    if (num > 3)
        return ESP_FAIL;

    return (i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV0_ADDR + num * 3, addr));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_address(mpu6050_dev_t *setting, uint8_t num, uint8_t address)
{
    if (num > 3)
        return ESP_FAIL;

    return i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV0_ADDR + num * 3, address);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_register(mpu6050_dev_t *setting, uint8_t num, uint8_t *reg)
{
    if (num > 3)
        return ESP_FAIL;

    return (i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV0_REG + num * 3, reg));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_register(mpu6050_dev_t *setting, uint8_t num, uint8_t reg)
{
    if (num > 3)
        return ESP_FAIL;

    return i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV0_REG + num * 3, reg);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_enabled(mpu6050_dev_t *setting, uint8_t num)
{
    uint8_t enabled = 0;
    if (num > 3)
        return ESP_FAIL;

    ESP_ERROR_RETURN(
        i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT, &(enabled)));

    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_enabled(mpu6050_dev_t *setting, uint8_t num, bool enabled)
{
    if (num > 3)
        return ESP_FAIL;

    return i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_word_byte_swap(mpu6050_dev_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 3)
        return ESP_FAIL;

    ESP_ERROR_RETURN(
        i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_BYTE_SW_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_word_byte_swap(mpu6050_dev_t *setting, uint8_t num, bool enabled)
{
    if (num > 3)
        return ESP_FAIL;

    return i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_BYTE_SW_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_write_mode(mpu6050_dev_t *setting, uint8_t num, bool *mode)
{
    if (num > 3)
        return ESP_FAIL;

    return (
        i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_REG_DIS_BIT, (uint8_t *)mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_write_mode(mpu6050_dev_t *setting, uint8_t num, bool mode)
{
    if (num > 3)
        return ESP_FAIL;

    return (i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_REG_DIS_BIT, mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_word_group_offset(mpu6050_dev_t *setting, uint8_t num, bool *enabled)
{
    if (num > 3)
        return ESP_FAIL;

    return (
        i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_GRP_BIT, (uint8_t *)enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_word_group_offset(mpu6050_dev_t *setting, uint8_t num, bool enabled)
{
    if (num > 3)
        return ESP_FAIL;

    return (i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_GRP_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_data_length(mpu6050_dev_t *setting, uint8_t num, uint8_t *length)
{
    if (num > 3)
        return ESP_FAIL;

    return (i2c_read_bits(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT,
        MPU6050_I2C_SLV_LEN_LENGTH, length));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_data_length(mpu6050_dev_t *setting, uint8_t num, uint8_t length)
{
    if (num > 3)
        return ESP_FAIL;

    return (i2c_write_bits(setting, MPU6050_REGISTER_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT,
        MPU6050_I2C_SLV_LEN_LENGTH, length));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_address(mpu6050_dev_t *setting, uint8_t *address)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV4_ADDR, address));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_4_address(mpu6050_dev_t *setting, uint8_t address)
{
    return i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV4_ADDR, address);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_slave_4_register(mpu6050_dev_t *setting, uint8_t *reg)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV4_REG, reg));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_4_register(mpu6050_dev_t *setting, uint8_t reg)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV4_REG, reg));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_4_output_byte(mpu6050_dev_t *setting, uint8_t data)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV4_DO, data));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_enabled(mpu6050_dev_t *setting, bool *enabled)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, (uint8_t *)enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_4_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_interrupt_enabled(mpu6050_dev_t *setting, bool *enabled)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, (uint8_t *)enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_4_interrupt_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_write_mode(mpu6050_dev_t *setting, uint8_t *mode)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, (uint8_t *)mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_4_write_mode(mpu6050_dev_t *setting, bool mode)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_master_delay(mpu6050_dev_t *setting, uint8_t *delay)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
        MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_4_master_delay(mpu6050_dev_t *setting, uint8_t delay)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
        MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_input_byte(mpu6050_dev_t *setting, uint8_t *byte)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_I2C_SLV4_DI, byte));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_passthrough_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_PASS_THROUGH_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_is_done(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_DONE_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_lost_arbitration(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_LOST_ARB_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_4_nack(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_NACK_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_3_nack(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV3_NACK_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_2_nack(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV2_NACK_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_1_nack(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV1_NACK_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_0_nack(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_STATUS, MPU6050_MST_I2C_SLV0_NACK_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_interrupt_mode(mpu6050_dev_t *setting, bool *mode)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, (uint8_t *)mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_interrupt_mode(mpu6050_dev_t *setting, bool mode)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_interrupt_drive(mpu6050_dev_t *setting, bool *drive)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, (uint8_t *)drive));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_interrupt_drive(mpu6050_dev_t *setting, bool drive)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_interrupt_latch(mpu6050_dev_t *setting, bool *latch)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, (uint8_t *)latch));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_interrupt_latch(mpu6050_dev_t *setting, bool latch)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_interrupt_latch_clear(mpu6050_dev_t *setting, bool *clear)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, (uint8_t *)clear));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_interrupt_latch_clear(mpu6050_dev_t *setting, bool clear)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fsync_interrupt_level(mpu6050_dev_t *setting, bool *level)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, (uint8_t *)level));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_fsync_interrupt_level(mpu6050_dev_t *setting, bool level)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fsync_interrupt_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_fsync_interrupt_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_i2c_bypass_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_i2c_bypass_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_clock_output_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_clock_output_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
uint8_t mpu6050_get_int_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_INT_ENABLE, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_enabled(mpu6050_dev_t *setting, uint8_t enabled)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_INT_ENABLE, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_freefall_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_freefall_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_motion_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_motion_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_zero_motion_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_zero_motion_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_fifo_byte_overflow_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_fifo_byte_overflow_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_i2c_master_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_i2c_master_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_data_ready_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_data_ready_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_INT_STATUS, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_freefall_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_FF_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_motion_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_MOT_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_zero_motion_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_ZMOT_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_fifo_byte_overflow_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_i2c_master_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_I2C_MST_INT_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_data_ready_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_acceleration(mpu6050_dev_t *setting, mpu6050_acceleration_t *accel)
{
    uint8_t buffer[6];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_XOUT_H, 6, (uint8_t *)buffer));
    accel->x = (((int16_t)buffer[0]) << 8) | buffer[1];
    accel->y = (((int16_t)buffer[2]) << 8) | buffer[3];
    accel->z = (((int16_t)buffer[4]) << 8) | buffer[5];
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_acceleration_x(mpu6050_dev_t *setting, int16_t *accel_x)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_XOUT_H, 2, buffer));

    *accel_x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_acceleration_y(mpu6050_dev_t *setting, int16_t *accel_y)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_YOUT_H, 2, buffer));
    *accel_y = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_acceleration_z(mpu6050_dev_t *setting, int16_t *accel_z)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_ZOUT_H, 2, buffer));
    *accel_z = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_temperature(mpu6050_dev_t *setting, int16_t *temp)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_TEMP_OUT_H, 2, buffer));

    int16_t rawtemp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    *temp = (rawtemp / 340.0) + 36.53;
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_rotation(mpu6050_dev_t *setting, mpu6050_rotation_t *rotat)
{
    uint8_t buffer[6];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_GYRO_XOUT_H, 6, buffer));
    rotat->x = (((int16_t)buffer[0]) << 8) | buffer[1];
    rotat->y = (((int16_t)buffer[2]) << 8) | buffer[3];
    rotat->z = (((int16_t)buffer[4]) << 8) | buffer[5];
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_rotation_x(mpu6050_dev_t *setting, int16_t *rotation_x)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_GYRO_XOUT_H, 2, buffer));
    *rotation_x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_rotation_y(mpu6050_dev_t *setting, int16_t *rotation_y)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_GYRO_YOUT_H, 2, buffer));
    *rotation_y = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_rotation_z(mpu6050_dev_t *setting, int16_t *rotation_z)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_GYRO_ZOUT_H, 2, buffer));
    *rotation_z = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_motion(mpu6050_dev_t *setting, mpu6050_acceleration_t *data_accel, mpu6050_rotation_t *data_gyro)
{
    uint8_t buffer[14];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_ACCEL_XOUT_H, 14, buffer));

    data_accel->x = (((int16_t)buffer[0]) << 8) | buffer[1];
    data_accel->y = (((int16_t)buffer[2]) << 8) | buffer[3];
    data_accel->z = (((int16_t)buffer[4]) << 8) | buffer[5];
    data_gyro->x = (((int16_t)buffer[8]) << 8) | buffer[9];
    data_gyro->y = (((int16_t)buffer[10]) << 8) | buffer[11];
    data_gyro->z = (((int16_t)buffer[12]) << 8) | buffer[13];
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_external_sensor_byte(mpu6050_dev_t *setting, int position, uint8_t *byte)
{
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_EXT_SENS_DATA_00 + position, byte));
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_external_sensor_word(mpu6050_dev_t *setting, int position, uint16_t *word)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_EXT_SENS_DATA_00 + position, 2, buffer));
    *word = ((((uint16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_external_sensor_dword(mpu6050_dev_t *setting, int position, uint32_t *dword)
{
    uint8_t buffer[4];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_EXT_SENS_DATA_00 + position, 4, buffer));
    *dword = ((((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) | (((uint16_t)buffer[2]) << 8) | buffer[3]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_motion_status(mpu6050_dev_t *setting, uint8_t *status)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, status));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_negative_motion_detected(mpu6050_dev_t *setting, uint8_t *motion)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XNEG_BIT, motion));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_positive_motion_detected(mpu6050_dev_t *setting, uint8_t *detected)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XPOS_BIT, detected));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_negative_motion_detected(mpu6050_dev_t *setting, uint8_t *detected)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YNEG_BIT, detected));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_positive_motion_detected(mpu6050_dev_t *setting, uint8_t *detected)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YPOS_BIT, detected));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_negative_motion_detected(mpu6050_dev_t *setting, uint8_t *detected)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZNEG_BIT, detected));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_positive_motion_detected(mpu6050_dev_t *setting, uint8_t *detected)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZPOS_BIT, detected));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_zero_motion_detected(mpu6050_dev_t *setting, uint8_t *detected)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZRMOT_BIT, detected));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_output_byte(mpu6050_dev_t *setting, uint8_t num, uint8_t data)
{
    if (num > 3)
        return ESP_FAIL;

    return (i2c_write_byte(setting, MPU6050_REGISTER_I2C_SLV0_DO + num, data));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_external_shadow_delay_enabled(mpu6050_dev_t *setting, bool *enabled)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT,
        (uint8_t *)enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_external_shadow_delay_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_slave_delay_enabled(mpu6050_dev_t *setting, uint8_t num)
{
    uint8_t byte = 0;
    if (num > 4)
        return ESP_FAIL;

    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, num, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_slave_delay_enabled(mpu6050_dev_t *setting, uint8_t num, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_I2C_MST_DELAY_CTRL, num, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_reset_gyroscope_path(mpu6050_dev_t *setting)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_GYRO_RESET_BIT, 1));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_reset_accelerometer_path(mpu6050_dev_t *setting)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_ACCEL_RESET_BIT, 1));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_reset_temperature_path(mpu6050_dev_t *setting)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_SIGNAL_PATH_RESET, MPU6050_PATHRESET_TEMP_RESET_BIT, 1));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_accelerometer_power_on_delay(mpu6050_dev_t *setting, uint8_t *delay)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_DELAY_BIT,
        MPU6050_DETECT_ACCEL_DELAY_LENGTH, delay));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_accelerometer_power_on_delay(mpu6050_dev_t *setting, uint8_t delay)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_DELAY_BIT,
        MPU6050_DETECT_ACCEL_DELAY_LENGTH, delay));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_freefall_detection_counter_decrement(mpu6050_dev_t *setting, uint8_t *decrement)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT,
        MPU6050_DETECT_FF_COUNT_LENGTH, decrement));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_freefall_detection_counter_decrement(mpu6050_dev_t *setting, uint8_t decrement)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT,
        MPU6050_DETECT_FF_COUNT_LENGTH, decrement));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_motion_detection_counter_decrement(mpu6050_dev_t *setting, uint8_t *decrement)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT,
        MPU6050_DETECT_MOT_COUNT_LENGTH, decrement));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_motion_detection_counter_decrement(mpu6050_dev_t *setting, uint8_t decrement)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT,
        MPU6050_DETECT_MOT_COUNT_LENGTH, decrement));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fifo_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, &(enabled)));
    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_fifo_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_i2c_master_mode_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, &(enabled)));
    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_i2c_master_mode_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled));
}

esp_err_t mpu6050_switch_spie_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_reset_fifo(mpu6050_dev_t *setting)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_reset_sensors(mpu6050_dev_t *setting)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT, 1));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_reset(mpu6050_dev_t *setting)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_sleep_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &(enabled)));
    return ((enabled == 0) ? DISABLE : ENABLE);
}

esp_err_t mpu6050_set_sleep_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_wake_cycle_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, &(enabled)));

    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_wake_cycle_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_temp_sensor_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, &(enabled)));

    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_temp_sensor_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_clock_source(mpu6050_dev_t *setting, uint8_t *clk_setting)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH,
        clk_setting));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_clock_source(mpu6050_dev_t *setting, uint8_t source)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH,
        source));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_wake_frequency(mpu6050_dev_t *setting, uint8_t *frequency)
{
    ESP_ERROR_RETURN(i2c_read_bits(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT,
        MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency));

    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_wake_frequency(mpu6050_dev_t *setting, uint8_t frequency)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT,
        MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_x_accel_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, &(enabled)));

    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_standby_x_accel_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_y_accel_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, &(enabled)));

    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_standby_y_accel_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_z_accel_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, &(enabled)));
    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_standby_z_accel_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_x_gyro_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, &(enabled)));
    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_standby_x_gyro_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_y_gyro_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, &(enabled)));

    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_standby_y_gyro_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_standby_z_gyro_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, &(enabled)));
    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_standby_z_gyro_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fifo_count(mpu6050_dev_t *setting, uint16_t *count)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_FIFO_COUNTH, 2, buffer));
    *count = ((((uint16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fifo_byte(mpu6050_dev_t *setting, uint8_t *byte)
{
    ESP_ERROR_RETURN(i2c_read_byte(setting, MPU6050_REGISTER_FIFO_R_W, byte));
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_fifo_bytes(mpu6050_dev_t *setting, uint8_t *data, uint8_t length)
{
    if (length <= 0)
    {
        *data = 0;
        return ESP_ERR_INVALID_ARG;
    }
    return (i2c_read_bytes(setting, MPU6050_REGISTER_FIFO_R_W, length, data));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_fifo_byte(mpu6050_dev_t *setting, uint8_t data)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_FIFO_R_W, data));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_device_id(mpu6050_dev_t *setting, uint8_t *id)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_device_id(mpu6050_dev_t *setting, uint8_t id)
{
    return (i2c_write_bits(setting, MPU6050_REGISTER_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_otp_bank_valid(mpu6050_dev_t *setting, uint8_t *valid)
{
    return (i2c_read_bit(setting, MPU6050_REGISTER_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, valid));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_otp_bank_valid(mpu6050_dev_t *setting, int8_t enabled)
{
    return (i2c_write_bit(setting, MPU6050_REGISTER_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_gyro_offset_tc(mpu6050_dev_t *setting, int8_t *offset)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH,
        (uint8_t *)offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_x_gyro_offset_tc(mpu6050_dev_t *setting, int8_t offset)
{
    return (
        i2c_write_bits(setting, MPU6050_REGISTER_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_gyro_offset_tc(mpu6050_dev_t *setting, int8_t *offset)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH,
        (uint8_t *)offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_y_gyro_offset_tc(mpu6050_dev_t *setting, int8_t offset)
{
    return (
        i2c_write_bits(setting, MPU6050_REGISTER_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_gyro_offset_tc(mpu6050_dev_t *setting, int8_t *offset)
{
    return (i2c_read_bits(setting, MPU6050_REGISTER_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH,
        (uint8_t *)offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_z_gyro_offset_tc(mpu6050_dev_t *setting, int8_t offset)
{
    return (
        i2c_write_bits(setting, MPU6050_REGISTER_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_fine_gain(mpu6050_dev_t *setting, int8_t *gain)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_X_FINE_GAIN, (uint8_t *)gain));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_x_fine_gain(mpu6050_dev_t *setting, int8_t gain)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_X_FINE_GAIN, gain));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_fine_gain(mpu6050_dev_t *setting, int8_t *gain)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_Y_FINE_GAIN, (uint8_t *)gain));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_y_fine_gain(mpu6050_dev_t *setting, int8_t gain)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_Y_FINE_GAIN, gain));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_fine_gain(mpu6050_dev_t *setting, int8_t *gain)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_Z_FINE_GAIN, (uint8_t *)gain));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_z_fine_gain(mpu6050_dev_t *setting, int8_t gain)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_Z_FINE_GAIN, gain));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_accel_offset(mpu6050_dev_t *setting, int16_t *offset)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_XA_OFFS_H, 2, buffer));
    *offset = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_x_accel_offset(mpu6050_dev_t *setting, int16_t offset)
{
    return (i2c_write_word(setting, MPU6050_REGISTER_XA_OFFS_H, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_accel_offset(mpu6050_dev_t *setting, int16_t *offset)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_YA_OFFS_H, 2, buffer));
    *offset = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_y_accel_offset(mpu6050_dev_t *setting, int16_t offset)
{
    return (i2c_write_word(setting, MPU6050_REGISTER_YA_OFFS_H, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_accel_offset(mpu6050_dev_t *setting, int16_t *offset)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_ZA_OFFS_H, 2, buffer));
    *offset = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_z_accel_offset(mpu6050_dev_t *setting, int16_t offset)
{
    return (i2c_write_word(setting, MPU6050_REGISTER_ZA_OFFS_H, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_x_gyro_offset(mpu6050_dev_t *setting, int16_t *offset)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_XG_OFFS_USRH, 2, buffer));
    *offset = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_x_gyro_offset(mpu6050_dev_t *setting, int16_t offset)
{
    return (i2c_write_word(setting, MPU6050_REGISTER_XG_OFFS_USRH, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_y_gyro_offset(mpu6050_dev_t *setting, int16_t *offset)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_YG_OFFS_USRH, 2, buffer));
    *offset = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_y_gyro_offset(mpu6050_dev_t *setting, int16_t offset)
{
    return (i2c_write_word(setting, MPU6050_REGISTER_YG_OFFS_USRH, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_z_gyro_offset(mpu6050_dev_t *setting, int16_t *offset)
{
    uint8_t buffer[2];
    ESP_ERROR_RETURN(i2c_read_bytes(setting, MPU6050_REGISTER_ZG_OFFS_USRH, 2, buffer));
    *offset = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_z_gyro_offset(mpu6050_dev_t *setting, int16_t offset)
{
    return (i2c_write_word(setting, MPU6050_REGISTER_ZG_OFFS_USRH, offset));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_pll_ready_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, &(enabled)));

    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_pll_ready_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_dmp_enabled(mpu6050_dev_t *setting)
{
    uint8_t enabled = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, &(enabled)));

    return ((enabled == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_int_dmp_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_5_status(mpu6050_dev_t *setting)
{
    uint8_t status = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_5_BIT, &(status)));

    return ((status == 0) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_4_status(mpu6050_dev_t *setting)
{
    uint8_t status = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_4_BIT, &(status)));

    return ((status == 0) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_3_status(mpu6050_dev_t *setting)
{
    uint8_t status = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_3_BIT, &(status)));

    return ((status == 0) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_2_status(mpu6050_dev_t *setting)
{
    uint8_t status = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_2_BIT, &(status)));

    return ((status == 0) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_1_status(mpu6050_dev_t *setting)
{
    uint8_t status = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_1_BIT, &(status)));

    return ((status == 0) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_int_0_status(mpu6050_dev_t *setting)
{
    uint8_t status = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_DMP_INT_STATUS, MPU6050_DMPINT_0_BIT, &(status)));

    return ((status == 0) ? ESP_FAIL : ESP_OK);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_ppl_ready_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_int_dmp_status(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_enabled(mpu6050_dev_t *setting)
{
    uint8_t byte = 0;
    ESP_ERROR_RETURN(i2c_read_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, &(byte)));

    return ((byte == 0) ? DISABLE : ENABLE);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_dmp_enabled(mpu6050_dev_t *setting, bool enabled)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_reset_dmp(mpu6050_dev_t *setting)
{
    return i2c_write_bit(setting, MPU6050_REGISTER_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_config_1(mpu6050_dev_t *setting, uint8_t *config)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_DMP_CFG_1, config));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_dmp_config_1(mpu6050_dev_t *setting, uint8_t config)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_DMP_CFG_1, config));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_get_dmp_config_2(mpu6050_dev_t *setting, uint8_t *config)
{
    return (i2c_read_byte(setting, MPU6050_REGISTER_DMP_CFG_2, config));
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_set_dmp_config_2(mpu6050_dev_t *setting, uint8_t config)
{
    return (i2c_write_byte(setting, MPU6050_REGISTER_DMP_CFG_2, config));
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
esp_err_t mpu6050_calibrate(mpu6050_dev_t *setting, float *accel_bias_res, float *gyro_bias_res)
{
    int16_t temp_offset[3];
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

    ESP_ERROR_CDEBUG(mpu6050_reset(setting));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CDEBUG(mpu6050_set_clock_source(setting, MPU6050_CLOCK_PLL_XGYRO));
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Configure device for bias calculation:
    ESP_ERROR_CDEBUG(mpu6050_set_int_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_accel_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_z_gyro_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_y_gyro_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_x_gyro_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_temp_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_clock_source(setting, MPU6050_CLOCK_INTERNAL));
    ESP_ERROR_CDEBUG(mpu6050_set_multi_master_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_i2c_master_mode_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_reset_sensors(setting));
    vTaskDelay(15 / portTICK_PERIOD_MS);

    // Configure MPU6050 gyro and accelerometer for bias calculation:
    ESP_ERROR_CDEBUG(mpu6050_set_rate(setting, false)); // Set sample rate to 1 kHz.
    ESP_ERROR_CDEBUG(mpu6050_set_dlpf_mode(setting, MPU6050_DLPF_BW_188));
    ESP_ERROR_CDEBUG(mpu6050_set_full_scale_accel_range(setting, MPU6050_ACCEL_FULL_SCALE_RANGE_2));
    ESP_ERROR_CDEBUG(mpu6050_set_full_scale_gyro_range(setting, MPU6050_GYRO_FULL_SCALE_RANGE_250));

    /**
     * Configure FIFO to capture data for bias calculation.
     */

    // Enable gyroscope and accelerometer sensors for FIFO:
    ESP_ERROR_CDEBUG(mpu6050_set_fifo_enabled(setting, true));
    ESP_ERROR_CDEBUG(mpu6050_set_accel_fifo_enabled(setting, true));
    ESP_ERROR_CDEBUG(mpu6050_set_z_gyro_fifo_enabled(setting, true));
    ESP_ERROR_CDEBUG(mpu6050_set_y_gyro_fifo_enabled(setting, true));
    ESP_ERROR_CDEBUG(mpu6050_set_x_gyro_fifo_enabled(setting, true));
    vTaskDelay(80 / portTICK_PERIOD_MS); // Accumulate 80 samples in 80 ms.

    // At end of sample accumulation, turn off FIFO sensor read:
    ESP_ERROR_CDEBUG(mpu6050_set_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_accel_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_z_gyro_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_y_gyro_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_x_gyro_fifo_enabled(setting, false));
    ESP_ERROR_CDEBUG(mpu6050_set_temp_fifo_enabled(setting, false));

    // Sets of full gyro and accelerometer data for averaging:
    ESP_ERROR_CDEBUG(mpu6050_get_fifo_count(setting, &(packet_count)));
    packet_count /= 12;

    for (int i = 0; i < packet_count; i++)
    {
        // Read data for averaging:
        ESP_ERROR_CDEBUG(mpu6050_get_fifo_bytes(setting, &tmp_data[0], 6));
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
    ESP_ERROR_CDEBUG(mpu6050_set_x_gyro_offset(setting, ((int16_t)tmp_data[0]) << 8 | tmp_data[1]));
    ESP_ERROR_CDEBUG(mpu6050_set_y_gyro_offset(setting, ((int16_t)tmp_data[2]) << 8 | tmp_data[3]));
    ESP_ERROR_CDEBUG(mpu6050_set_z_gyro_offset(setting, ((int16_t)tmp_data[4]) << 8 | tmp_data[5]));

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
    ESP_ERROR_CDEBUG(mpu6050_get_x_accel_offset(setting, &(temp_offset[0])));
    ESP_ERROR_CDEBUG(mpu6050_get_y_accel_offset(setting, &(temp_offset[1])));
    ESP_ERROR_CDEBUG(mpu6050_get_z_accel_offset(setting, &(temp_offset[2])));

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

    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_self_test(mpu6050_dev_t *setting, float *destination)
{
    uint8_t self_test[6];
    float factory_trim[6];

    // Configure the accelerometer for self-test:
    ESP_ERROR_CDEBUG(mpu6050_set_accel_x_self_test(setting, true));
    ESP_ERROR_CDEBUG(mpu6050_set_accel_y_self_test(setting, true));
    ESP_ERROR_CDEBUG(mpu6050_set_accel_z_self_test(setting, true));
    ESP_ERROR_CDEBUG(mpu6050_set_full_scale_accel_range(setting, MPU6050_ACCEL_FULL_SCALE_RANGE_8));
    ESP_ERROR_CDEBUG(mpu6050_set_full_scale_gyro_range(setting, MPU6050_GYRO_FULL_SCALE_RANGE_250));

    mpu6050_get_accel_x_self_test_factory_trim(setting, &(self_test[0]));
    mpu6050_get_accel_y_self_test_factory_trim(setting, &(self_test[1]));
    mpu6050_get_accel_z_self_test_factory_trim(setting, &(self_test[2]));
    mpu6050_get_gyro_x_self_test_factory_trim(setting, &(self_test[3]));
    mpu6050_get_gyro_y_self_test_factory_trim(setting, &(self_test[4]));
    mpu6050_get_gyro_z_self_test_factory_trim(setting, &(self_test[5]));

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
    {
        destination[i] = 100.0f + 100.0f * (self_test[i] - factory_trim[i]) / factory_trim[i];
    }
    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_madgwick_quaternion_update(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y,
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
        return ESP_FAIL;

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

    return ESP_OK;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_test_connection(mpu6050_dev_t *setting)
{
    uint8_t id;
    return (((mpu6050_get_device_id(setting, &(id)) == ESP_OK) && (id == 0x34)) ? ESP_OK : ESP_FAIL);
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_init_desc(mpu6050_dev_t *mpu6050_config, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio,
    gpio_num_t scl_gpio)
{
    ESP_PARAM_CHECK(mpu6050_config);

    esp_err_t ret = ESP_OK;

    if ((addr != MPU6050_ADDRESS_LOW) && (addr != MPU6050_ADDRESS_HIGH))
    {
        ESP_LOGE(TAG, "Invalid device address: 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }
    // Init i2cdev library
    ESP_ERROR_RETURN(i2cdev_init());

    mpu6050_config->port = port;
    mpu6050_config->addr = addr;
    mpu6050_config->cfg.sda_io_num = sda_gpio;
    mpu6050_config->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    mpu6050_config->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    ret = i2c_dev_probe(mpu6050_config, I2C_DEV_READ);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "[%s, %d] device not available <%s>", __func__, __LINE__, esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_dev_create_mutex(mpu6050_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "[%s, %d] failed to create mutex for i2c dev. <%s>", __func__, __LINE__, esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CDEBUG(mpu6050_set_clock_source(mpu6050_config, MPU6050_CLOCK_PLL_XGYRO));
    ESP_ERROR_CDEBUG(mpu6050_set_full_scale_gyro_range(mpu6050_config, MPU6050_GYRO_FULL_SCALE_RANGE_250));
    ESP_ERROR_CDEBUG(mpu6050_set_full_scale_accel_range(mpu6050_config, MPU6050_ACCEL_FULL_SCALE_RANGE_2));
    ESP_ERROR_CDEBUG(mpu6050_set_sleep_enabled(mpu6050_config, false));

    return ret;
}
/* ------------------------------------------------------ */
esp_err_t mpu6050_deinit_desc(mpu6050_dev_t *mpu6050_config)
{
    ESP_PARAM_CHECK(mpu6050_config);

    ESP_ERROR_RETURN(i2cdev_done());
    return (i2c_dev_delete_mutex(mpu6050_config));
}
/* ------------------------------------------------------ */
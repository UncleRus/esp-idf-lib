

#ifndef MPU6050_H
#define MPU6050_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "i2c_dev_bridge.h"

// Address of MPU6050 (Can be 0x68 or 0x69):
#define MPU6050_ADDRESS_LOW  (0x68) // Address pin low (GND).
#define MPU6050_ADDRESS_HIGH (0x69) // Address pin high (VCC).

/**
 * DLPF values:
 */
#define MPU6050_DLPF_BW_256                 (0x00)
#define MPU6050_DLPF_BW_188                 (0x01)
#define MPU6050_DLPF_BW_98                  (0x02)
#define MPU6050_DLPF_BW_42                  (0x03)
#define MPU6050_DLPF_BW_20                  (0x04)
#define MPU6050_DLPF_BW_10                  (0x05)
#define MPU6050_DLPF_BW_5                   (0x06)

/**
 * DHPF values:
 */
#define MPU6050_DHPF_RESET                  (0x00)
#define MPU6050_DHPF_5                      (0x01)
#define MPU6050_DHPF_2P5                    (0x02)
#define MPU6050_DHPF_1P25                   (0x03)
#define MPU6050_DHPF_0P63                   (0x04)
#define MPU6050_DHPF_HOLD                   (0x07)

/**
 * Full scale gyroscope range:
 */
#define MPU6050_GYRO_FULL_SCALE_RANGE_250   (0x00)
#define MPU6050_GYRO_FULL_SCALE_RANGE_500   (0x01)
#define MPU6050_GYRO_FULL_SCALE_RANGE_1000  (0x02)
#define MPU6050_GYRO_FULL_SCALE_RANGE_2000  (0x03)

/**
 * Full scale accelerometer range:
 */
#define MPU6050_ACCEL_FULL_SCALE_RANGE_2    (0x00)
#define MPU6050_ACCEL_FULL_SCALE_RANGE_4    (0x01)
#define MPU6050_ACCEL_FULL_SCALE_RANGE_8    (0x02)
#define MPU6050_ACCEL_FULL_SCALE_RANGE_16   (0x03)

/**
 * Interrupt values:
 */
#define MPU6050_INTMODE_ACTIVEHIGH          (0x00)
#define MPU6050_INTMODE_ACTIVELOW           (0x01)
#define MPU6050_INTDRV_PUSHPULL             (0x00)
#define MPU6050_INTDRV_OPENDRAIN            (0x01)
#define MPU6050_INTLATCH_50USPULSE          (0x00)
#define MPU6050_INTLATCH_WAITCLEAR          (0x01)
#define MPU6050_INTCLEAR_STATUSREAD         (0x00)
#define MPU6050_INTCLEAR_ANYREAD            (0x01)

/**
 * Clock sources:
 */
#define MPU6050_CLOCK_INTERNAL              (0x00)
#define MPU6050_CLOCK_PLL_XGYRO             (0x01)
#define MPU6050_CLOCK_PLL_YGYRO             (0x02)
#define MPU6050_CLOCK_PLL_ZGYRO             (0x03)
#define MPU6050_CLOCK_PLL_EXTERNAL_32K      (0x04)
#define MPU6050_CLOCK_PLL_EXTERNAL_19M      (0x05)
#define MPU6050_CLOCK_KEEP_RESET            (0x07)

/**
 * Wake frequencies:
 */
#define MPU6050_WAKE_FREQ_1P25              (0x0)
#define MPU6050_WAKE_FREQ_2P5               (0x1)
#define MPU6050_WAKE_FREQ_5                 (0x2)
#define MPU6050_WAKE_FREQ_10                (0x3)

/**
 * Decrement values:
 */
#define MPU6050_DETECT_DECREMENT_RESET      (0x0)
#define MPU6050_DETECT_DECREMENT_1          (0x1)
#define MPU6050_DETECT_DECREMENT_2          (0x2)
#define MPU6050_DETECT_DECREMENT_4          (0x3)

/**
 * External sync values:
 */
#define MPU6050_EXT_SYNC_DISABLED           (0x0)
#define MPU6050_EXT_SYNC_TEMP_OUT_L         (0x1)
#define MPU6050_EXT_SYNC_GYRO_XOUT_L        (0x2)
#define MPU6050_EXT_SYNC_GYRO_YOUT_L        (0x3)
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L        (0x4)
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L       (0x5)
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L       (0x6)
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L       (0x7)

/**
 * Clock division values:
 */
#define MPU6050_CLOCK_DIV_348               (0x0)
#define MPU6050_CLOCK_DIV_333               (0x1)
#define MPU6050_CLOCK_DIV_320               (0x2)
#define MPU6050_CLOCK_DIV_308               (0x3)
#define MPU6050_CLOCK_DIV_296               (0x4)
#define MPU6050_CLOCK_DIV_286               (0x5)
#define MPU6050_CLOCK_DIV_276               (0x6)
#define MPU6050_CLOCK_DIV_267               (0x7)
#define MPU6050_CLOCK_DIV_258               (0x8)
#define MPU6050_CLOCK_DIV_500               (0x9)
#define MPU6050_CLOCK_DIV_471               (0xA)
#define MPU6050_CLOCK_DIV_444               (0xB)
#define MPU6050_CLOCK_DIV_421               (0xC)
#define MPU6050_CLOCK_DIV_400               (0xD)
#define MPU6050_CLOCK_DIV_381               (0xE)
#define MPU6050_CLOCK_DIV_364               (0xF)

/**
 * Bit and length defines for SELF_TEST register:
 */
#define MPU6050_SELF_TEST_XA_1_BIT          (0x07)
#define MPU6050_SELF_TEST_XA_1_LENGTH       (0x03)
#define MPU6050_SELF_TEST_XA_2_BIT          (0x05)
#define MPU6050_SELF_TEST_XA_2_LENGTH       (0x02)
#define MPU6050_SELF_TEST_YA_1_BIT          (0x07)
#define MPU6050_SELF_TEST_YA_1_LENGTH       (0x03)
#define MPU6050_SELF_TEST_YA_2_BIT          (0x03)
#define MPU6050_SELF_TEST_YA_2_LENGTH       (0x02)
#define MPU6050_SELF_TEST_ZA_1_BIT          (0x07)
#define MPU6050_SELF_TEST_ZA_1_LENGTH       (0x03)
#define MPU6050_SELF_TEST_ZA_2_BIT          (0x01)
#define MPU6050_SELF_TEST_ZA_2_LENGTH       (0x02)
#define MPU6050_SELF_TEST_XG_1_BIT          (0x04)
#define MPU6050_SELF_TEST_XG_1_LENGTH       (0x05)
#define MPU6050_SELF_TEST_YG_1_BIT          (0x04)
#define MPU6050_SELF_TEST_YG_1_LENGTH       (0x05)
#define MPU6050_SELF_TEST_ZG_1_BIT          (0x04)
#define MPU6050_SELF_TEST_ZG_1_LENGTH       (0x05)

/**
 * Bit and length defines for CONFIG register:
 */
#define MPU6050_CFG_EXT_SYNC_SET_BIT        (5)
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH     (3)
#define MPU6050_CFG_DLPF_CFG_BIT            (2)
#define MPU6050_CFG_DLPF_CFG_LENGTH         (3)

/**
 * Bit and length defines for GYRO_CONFIG register:
 */
#define MPU6050_GCONFIG_FS_SEL_BIT          (4)
#define MPU6050_GCONFIG_FS_SEL_LENGTH       (2)

/**
 * Bit and length defines for ACCEL_CONFIG register:
 */
#define MPU6050_ACONFIG_XA_ST_BIT           (7)
#define MPU6050_ACONFIG_YA_ST_BIT           (6)
#define MPU6050_ACONFIG_ZA_ST_BIT           (5)
#define MPU6050_ACONFIG_AFS_SEL_BIT         (4)
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      (2)
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       (2)
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    (3)

/**
 * Bit and length defines for FIFO_EN register:
 */
#define MPU6050_TEMP_FIFO_EN_BIT            (7)
#define MPU6050_XG_FIFO_EN_BIT              (6)
#define MPU6050_YG_FIFO_EN_BIT              (5)
#define MPU6050_ZG_FIFO_EN_BIT              (4)
#define MPU6050_ACCEL_FIFO_EN_BIT           (3)
#define MPU6050_SLV2_FIFO_EN_BIT            (2)
#define MPU6050_SLV1_FIFO_EN_BIT            (1)
#define MPU6050_SLV0_FIFO_EN_BIT            (0)

/**
 * Bit and length defines for I2C_MST_CTRL register:
 */
#define MPU6050_MULT_MST_EN_BIT             (7)
#define MPU6050_WAIT_FOR_ES_BIT             (6)
#define MPU6050_SLV_3_FIFO_EN_BIT           (5)
#define MPU6050_I2C_MST_P_NSR_BIT           (4)
#define MPU6050_I2C_MST_CLK_BIT             (3)
#define MPU6050_I2C_MST_CLK_LENGTH          (4)

/**
 * Bit and length defines for I2C_SLV* register:
 */
#define MPU6050_I2C_SLV_RW_BIT              (7)
#define MPU6050_I2C_SLV_ADDR_BIT            (6)
#define MPU6050_I2C_SLV_ADDR_LENGTH         (7)
#define MPU6050_I2C_SLV_EN_BIT              (7)
#define MPU6050_I2C_SLV_BYTE_SW_BIT         (6)
#define MPU6050_I2C_SLV_REG_DIS_BIT         (5)
#define MPU6050_I2C_SLV_GRP_BIT             (4)
#define MPU6050_I2C_SLV_LEN_BIT             (3)
#define MPU6050_I2C_SLV_LEN_LENGTH          (4)

/**
 * Bit and length defines for I2C_SLV4 register:
 */
#define MPU6050_I2C_SLV4_RW_BIT             (7)
#define MPU6050_I2C_SLV4_ADDR_BIT           (6)
#define MPU6050_I2C_SLV4_ADDR_LENGTH        (7)
#define MPU6050_I2C_SLV4_EN_BIT             (7)
#define MPU6050_I2C_SLV4_INT_EN_BIT         (6)
#define MPU6050_I2C_SLV4_REG_DIS_BIT        (5)
#define MPU6050_I2C_SLV4_MST_DLY_BIT        (4)
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH     (5)

/**
 * Bit and length defines for I2C_MST_STATUS register:
 */
#define MPU6050_MST_PASS_THROUGH_BIT        (7)
#define MPU6050_MST_I2C_SLV4_DONE_BIT       (6)
#define MPU6050_MST_I2C_LOST_ARB_BIT        (5)
#define MPU6050_MST_I2C_SLV4_NACK_BIT       (4)
#define MPU6050_MST_I2C_SLV3_NACK_BIT       (3)
#define MPU6050_MST_I2C_SLV2_NACK_BIT       (2)
#define MPU6050_MST_I2C_SLV1_NACK_BIT       (1)
#define MPU6050_MST_I2C_SLV0_NACK_BIT       (0)

/**
 * Bit and length defines for INT_PIN_CFG register:
 */
#define MPU6050_INTCFG_INT_LEVEL_BIT        (7)
#define MPU6050_INTCFG_INT_OPEN_BIT         (6)
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     (5)
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     (4)
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  (3)
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     (2)
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    (1)
#define MPU6050_INTCFG_CLKOUT_EN_BIT        (0)

/**
 * Bit and length defines for INT_ENABLE and INT_STATUS registers:
 */
#define MPU6050_INTERRUPT_FF_BIT            (7)
#define MPU6050_INTERRUPT_MOT_BIT           (6)
#define MPU6050_INTERRUPT_ZMOT_BIT          (5)
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    (4)
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   (3)
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   (2)
#define MPU6050_INTERRUPT_DMP_INT_BIT       (1)
#define MPU6050_INTERRUPT_DATA_RDY_BIT      (0)

/**
 * Bit and length defines for MOT_DETECT_STATUS register:
 */
#define MPU6050_MOTION_MOT_XNEG_BIT         (7)
#define MPU6050_MOTION_MOT_XPOS_BIT         (6)
#define MPU6050_MOTION_MOT_YNEG_BIT         (5)
#define MPU6050_MOTION_MOT_YPOS_BIT         (4)
#define MPU6050_MOTION_MOT_ZNEG_BIT         (3)
#define MPU6050_MOTION_MOT_ZPOS_BIT         (2)
#define MPU6050_MOTION_MOT_ZRMOT_BIT        (0)

/**
 * Bit and length defines for I2C_MST_DELAY_CTRL register:
 */
#define MPU6050_DLYCTRL_DELAY_ES_SHADOW_BIT (7)
#define MPU6050_DLYCTRL_I2C_SLV4_DLY_EN_BIT (4)
#define MPU6050_DLYCTRL_I2C_SLV3_DLY_EN_BIT (3)
#define MPU6050_DLYCTRL_I2C_SLV2_DLY_EN_BIT (2)
#define MPU6050_DLYCTRL_I2C_SLV1_DLY_EN_BIT (1)
#define MPU6050_DLYCTRL_I2C_SLV0_DLY_EN_BIT (0)

/**
 * Bit and length defines for SIGNAL_PATH_RESET register:
 */
#define MPU6050_PATHRESET_GYRO_RESET_BIT    (2)
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   (1)
#define MPU6050_PATHRESET_TEMP_RESET_BIT    (0)

/**
 * Bit and length defines for MOT_DETECT_CTRL register:
 */
#define MPU6050_DETECT_ACCEL_DELAY_BIT      (5)
#define MPU6050_DETECT_ACCEL_DELAY_LENGTH   (2)
#define MPU6050_DETECT_FF_COUNT_BIT         (3)
#define MPU6050_DETECT_FF_COUNT_LENGTH      (2)
#define MPU6050_DETECT_MOT_COUNT_BIT        (1)
#define MPU6050_DETECT_MOT_COUNT_LENGTH     (2)

/**
 * Bit and length defines for USER_CTRL register:
 */
#define MPU6050_USERCTRL_DMP_EN_BIT         (7)
#define MPU6050_USERCTRL_FIFO_EN_BIT        (6)
#define MPU6050_USERCTRL_I2C_MST_EN_BIT     (5)
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT     (4)
#define MPU6050_USERCTRL_DMP_RESET_BIT      (3)
#define MPU6050_USERCTRL_FIFO_RESET_BIT     (2)
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT  (1)
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT (0)

/**
 * Bit and length defines for PWR_MGMT_1 register:
 */
#define MPU6050_PWR1_DEVICE_RESET_BIT       (7)
#define MPU6050_PWR1_SLEEP_BIT              (6)
#define MPU6050_PWR1_CYCLE_BIT              (5)
#define MPU6050_PWR1_TEMP_DIS_BIT           (3)
#define MPU6050_PWR1_CLKSEL_BIT             (2)
#define MPU6050_PWR1_CLKSEL_LENGTH          (3)

/**
 * Bit and length defines for PWR_MGMT_2 register:
 */
#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       (7)
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    (2)
#define MPU6050_PWR2_STBY_XA_BIT            (5)
#define MPU6050_PWR2_STBY_YA_BIT            (4)
#define MPU6050_PWR2_STBY_ZA_BIT            (3)
#define MPU6050_PWR2_STBY_XG_BIT            (2)
#define MPU6050_PWR2_STBY_YG_BIT            (1)
#define MPU6050_PWR2_STBY_ZG_BIT            (0)

/**
 * Bit and length defines for WHO_AM_I register:
 */
#define MPU6050_WHO_AM_I_BIT                (6)
#define MPU6050_WHO_AM_I_LENGTH             (6)

/**
 * Undocumented bits and lengths:
 */
#define MPU6050_TC_PWR_MODE_BIT             (7)
#define MPU6050_TC_OFFSET_BIT               (6)
#define MPU6050_TC_OFFSET_LENGTH            (6)
#define MPU6050_TC_OTP_BNK_VLD_BIT          (0)
#define MPU6050_DMPINT_5_BIT                (5)
#define MPU6050_DMPINT_4_BIT                (4)
#define MPU6050_DMPINT_3_BIT                (3)
#define MPU6050_DMPINT_2_BIT                (2)
#define MPU6050_DMPINT_1_BIT                (1)
#define MPU6050_DMPINT_0_BIT                (0)

/**
 * @brief store mpu6050 acceleration data.
 * 
 */
typedef struct _mpu6050_acceleration_t
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} mpu6050_acceleration_t;

/**
 * @brief store mpu6050 rotation data.
 * 
 */
typedef struct _mpu6050_rotation_t
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_rotation_t;

/**
 * @brief mpu6050 config.
 * 
 */
typedef i2c_dev_t mpu6050_config_t;

/**
 * @brief Init mpu6050 i2c dev.
 * 
 * @param[in] setting: pointer to mpu6050 configuration
 * @return 
 *      - ESP_OK: i2c dev initialized successfully.
 *      - ESP_FAIL: failed to initialized.
 *      - ESP_ERR_INVALID_ARG: invalid argument
 *      - ESP_ERROR_TIMEOUT: failed to read the mpu6050 sensor.
 */
esp_err_t mpu6050_init(mpu6050_config_t *mpu6050_config);

/**
 * @brief Deinit mpu6050 i2c dev.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: i2c dev initialized successfully.
 *      - ESP_ERR_INVALID_ARG: invalid argument 
 */
esp_err_t mpu6050_deinit(mpu6050_config_t *mpu6050_config);

/**
 * @brief Verify the I2C connection. Make sure the device is connected and
 * responds as expected.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: connection is valid else ESP_FAIL
 */
esp_err_t mpu6050_test_connection(const mpu6050_config_t *setting);

/**
 * @brief Get the auxiliary I2C supply voltage level.
 * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
 * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
 * the MPU-6000, which does not have a VLOGIC pin.
 *
 * I2C supply voltage level
 * @return 
 *      - 0 : VLOGIC
 *      - 1 : VDD
 */     
uint8_t mpu6050_get_aux_vddio_level(const mpu6050_config_t *setting);

/**
 * @brief Set the auxiliary I2C supply voltage level.
 * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
 * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
 * the MPU-6000, which does not have a VLOGIC pin.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] level I2C supply voltage level (0 = VLOGIC, 1 = VDD).
 */
void mpu6050_set_aux_vddio_level(const mpu6050_config_t *setting, uint8_t level);

/**
 * @brief Get gyroscope output rate divider. The sensor register output,
 * FIFO output, DMP sampling, Motion Detection, Zero Motion Detection,
 * and Free Fall Detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * Where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current sample rate.
 */
uint8_t mpu6050_get_rate(const mpu6050_config_t *setting);

/**
 * @brief Set gyroscope output rate divider.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] rate New sample rate divider.
 */
void mpu6050_set_rate(const mpu6050_config_t *setting, uint8_t rate);

/**
 * @brief Get external FSYNC configuration. Configures the external
 * Frame Synchronization (FSYNC) pin sampling. An external signal connected to
 * the FSYNC pin can be sampled by configuring EXT_SYNC_SET. Signal changes to
 * the FSYNC pin are latched so that short strobes may be captured. The latched
 * FSYNC signal will be sampled at the Sampling Rate, as defined in register 25.
 * After sampling, the latch will reset to the current FSYNC signal state.
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table:
 *
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 *  0           | Input disabled
 *  1           | TEMP_OUT_L   [0]
 *  2           | GYRO_XOUT_L  [0]
 *  3           | GYRO_YOUT_L  [0]
 *  4           | GYRO_ZOUT_L  [0]
 *  5           | ACCEL_XOUT_L [0]
 *  6           | ACCEL_YOUT_L [0]
 *  7           | ACCEL_ZOUT_L [0]
 * -------------+-------------------
 * @param[in] setting: pointer to mpu6050 setting
 * @return FSYNC configuration value.
 */
uint8_t mpu6050_get_external_frame_sync(const mpu6050_config_t *setting);

/**
 * @brief Set external FSYNC configuration.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in]  sync New FSYNC configuration value.
 */
void mpu6050_set_external_frame_sync(const mpu6050_config_t *setting, uint8_t sync);

/**
 * @brief: Get digital low-pass filter configuration. The DLPF_CFG parameter
 * sets the digital low pass filter configuration. It also determines the
 * internal sampling rate used by the device as shown in the table below.
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 *  0       | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 *  1       | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 *  2       | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 *  3       | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 *  4       | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 *  5       | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 *  6       | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 *  7       |      Reserved      |      Reserved      | Reserved
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return DLFP configuration.
 */
uint8_t mpu6050_get_dlpf_mode(const mpu6050_config_t *setting);

/**
 * @brief Set digital low-pass filter configuration.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in]  mode New DLFP configuration setting.
 */
void mpu6050_set_dlpf_mode(const mpu6050_config_t *setting, uint8_t mode);

/**
 * @brief Get full-scale gyroscope range. The FS_SEL parameter allows setting
 * the full-scale range of the gyro sensors, as described below:
 *
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current full-scale gyroscope range setting.
 */
uint8_t mpu6050_get_full_scale_gyro_range(const mpu6050_config_t *setting);

/**
 * @brief Set full-scale gyroscope range.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in]  range New full-scale gyroscope range value.
 */
void mpu6050_set_full_scale_gyro_range(const mpu6050_config_t *setting, uint8_t range);

/**
 * @brief Get self-test factory trim value for accelerometer X axis.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return Factory trim value.
 */
uint8_t mpu6050_get_accel_x_self_test_factory_trim(const mpu6050_config_t *setting);

/**
 * @brief Get self-test factory trim value for accelerometer Y axis.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return Factory trim value.
 */
uint8_t mpu6050_get_accel_y_self_test_factory_trim(const mpu6050_config_t *setting);

/**
 * @brief Get self-test factory trim value for accelerometer Z axis.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Factory trim value.
 */
uint8_t mpu6050_get_accel_z_self_test_factory_trim(const mpu6050_config_t *setting);

/**
 * @brief Get self-test factory trim value for gyroscope X axis.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Factory trim value.
 */
uint8_t mpu6050_get_gyro_x_self_test_factory_trim(const mpu6050_config_t *setting);

/**
 * @brief Get self-test factory trim value for gyroscope Y axis.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Factory trim value.
 */
uint8_t mpu6050_get_gyro_y_self_test_factory_trim(const mpu6050_config_t *setting);

/**
 * @brief Get self-test factory trim value for gyroscope Z axis.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Factory trim value.
 */
uint8_t mpu6050_get_gyro_z_self_test_factory_trim(const mpu6050_config_t *setting);

/**
 * @brief Get self-test enabled setting for accelerometer X axis.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable 
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_accel_x_self_test(const mpu6050_config_t *setting);

/**
 * @brief Set self-test enabled setting for accelerometer X axis.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled Self test enabled value.
 */
void mpu6050_set_accel_x_self_test(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get self-test enabled setting for accelerometer Y axis.
 * @param[in] setting: pointer to mpu6050 setting
 * 
 * @return 
 *      - ESP_OK: enable 
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_accel_y_self_test(const mpu6050_config_t *setting);

/**
 * @brief Set self-test enabled setting for accelerometer Y axis.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled Self test enabled value.
 */
void mpu6050_set_accel_y_self_test(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get self-test enabled setting for accelerometer Z axis.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable 
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_accel_z_self_test(const mpu6050_config_t *setting);

/**
 * @brief Set self-test enabled setting for accelerometer Z axis.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled Self test enabled value.
 */
void mpu6050_set_accel_z_self_test(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described below:
 *
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current full-scale accelerometer range setting.
 */
uint8_t mpu6050_get_full_scale_accel_range(const mpu6050_config_t *setting);

/**
 * @brief Set full-scale accelerometer range.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] range New full-scale accelerometer range setting.
 */
void mpu6050_set_full_scale_accel_range(const mpu6050_config_t *setting, uint8_t range);

/**
 * @brief Get the high-pass filter configuration.
 * The DHPF is a filter module in the path leading to motion detectors (Free
 * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
 * available to the data registers.
 *
 * The high pass filter has three modes:
 *
 *    Reset: The filter output settles to zero within one sample. This
 *           effectively disables the high pass filter. This mode may be toggled
 *           to quickly settle the filter.
 *
 *    On:    The high pass filter will pass signals above the cut off frequency.
 *
 *    Hold:  When triggered, the filter holds the present sample. The filter
 *           output will be the difference between the input sample and the held
 *           sample.
 *
 * ACCEL_HPF | Filter Mode | Cut-off Frequency
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current high-pass filter configuration.
 */
uint8_t mpu6050_get_dhpf_mode(const mpu6050_config_t *setting);

/**
 * @brief Set the high-pass filter configuration.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] mode New high-pass filter configuration.
 */
void mpu6050_set_dhpf_mode(const mpu6050_config_t *setting, uint8_t mode);

/**
 * @brief Get free-fall event acceleration threshold.
 * This register configures the detection threshold for Free Fall event
 * detection. The unit of FF_THR is 1LSB = 2mg. Free Fall is detected when the
 * absolute value of the accelerometer measurements for the three axes are each
 * less than the detection threshold. This condition increments the Free Fall
 * duration counter (Register 30). The Free Fall interrupt is triggered when the
 * Free Fall duration counter reaches the time specified in FF_DUR.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current free-fall acceleration threshold value (LSB = 2mg).
 */
uint8_t mpu6050_get_freefall_detection_threshold(const mpu6050_config_t *setting);

/**
 * @brief Get free-fall event acceleration threshold.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] threshold New free-fall acceleration threshold value (LSB = 2mg).
 */
void mpu6050_set_freefall_detection_threshold(const mpu6050_config_t *setting, uint8_t threshold);

/**
 * @brief Get free-fall event duration threshold.
 * This register configures the duration counter threshold for Free Fall event
 * detection. The duration counter ticks at 1kHz, therefore FF_DUR has a unit
 * of 1 LSB = 1 ms.
 *
 * The Free Fall duration counter increments while the absolute value of the
 * accelerometer measurements are each less than the detection threshold
 * (Register 29). The Free Fall interrupt is triggered when the Free Fall
 * duration counter reaches the time specified in this register.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current free-fall duration threshold value (LSB = 1ms).
 */
uint8_t mpu6050_get_freefall_detection_duration(const mpu6050_config_t *setting);

/**
 * @brief Get free-fall event duration threshold.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] duration New free-fall duration threshold value (LSB = 1ms).
 */
void mpu6050_set_freefall_detection_duration(const mpu6050_config_t *setting, uint8_t duration);

/**
 * @brief Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
 * absolute value of any of the accelerometer measurements exceeds this Motion
 * detection threshold. This condition increments the Motion detection duration
 * counter (Register 32). The Motion detection interrupt is triggered when the
 * Motion Detection counter reaches the time count specified in MOT_DUR
 * (Register 32).
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in MOT_DETECT_STATUS (Register 97).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current motion detection acceleration threshold value (LSB = 2mg).
 */
uint8_t mpu6050_get_motion_detection_threshold(const mpu6050_config_t *setting);

/**
 * @brief Set motion detection event acceleration threshold.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] threshold New motion detection acceleration threshold
 * value (LSB = 2mg).
 */
void mpu6050_set_motion_detection_threshold(const mpu6050_config_t *setting, uint8_t threshold);

/**
 * @brief Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
 * of 1LSB = 1ms. The Motion detection duration counter increments when the
 * absolute value of any of the accelerometer measurements exceeds the Motion
 * detection threshold (Register 31). The Motion detection interrupt is
 * triggered when the Motion detection counter reaches the time count specified
 * in this register.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current motion detection duration threshold value (LSB = 1ms).
 */
uint8_t mpu6050_get_motion_detection_duration(const mpu6050_config_t *setting);

/**
 * @brief Set motion detection event duration threshold.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] duration New motion detection duration threshold value (LSB = 1ms).
 */
void mpu6050_set_motion_detection_duration(const mpu6050_config_t *setting, uint8_t duration);

/**
 * @brief Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
 * the absolute value of the accelerometer measurements for the 3 axes are each
 * less than the detection threshold. This condition increments the Zero Motion
 * duration counter (Register 34). The Zero Motion interrupt is triggered when
 * the Zero Motion duration counter reaches the time count specified in
 * ZRMOT_DUR (Register 34).
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
 * interrupt both when Zero Motion is first detected and when Zero Motion is no
 * longer detected.
 *
 * When a zero motion event is detected, a Zero Motion Status will be indicated
 * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
 * condition is detected, the status bit is set to 1. When a zero-motion-to-
 * motion condition is detected, the status bit is set to 0.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current zero motion detection acceleration threshold
 * value (LSB = 2mg).
 */
uint8_t mpu6050_get_zero_motion_detection_threshold(const mpu6050_config_t *setting);

/**
 * @brief Set zero motion detection event acceleration threshold.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] threshold New zero motion detection acceleration threshold
 * value (LSB = 2mg).
 */
void mpu6050_set_zero_motion_detection_threshold(const mpu6050_config_t *setting, uint8_t threshold);

/**
 * @brief Get zero motion detection event duration threshold.
 * This register configures the duration counter threshold for Zero Motion
 * interrupt generation. The duration counter ticks at 16 Hz, therefore
 * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
 * increments while the absolute value of the accelerometer measurements are
 * each less than the detection threshold (Register 33). The Zero Motion
 * interrupt is triggered when the Zero Motion duration counter reaches the time
 * count specified in this register.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current zero motion detection duration threshold value (LSB = 64ms).
 */
uint8_t mpu6050_get_zero_motion_detection_duration(const mpu6050_config_t *setting);

/**
 * @brief Set zero motion detection event duration threshold.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] duration New zero motion detection duration threshold
 * value (LSB = 1ms).
 */
void mpu6050_set_zero_motion_detection_duration(const mpu6050_config_t *setting, uint8_t duration);

/**
 * @brief Get temperature FIFO enabled value.
 * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
 * 66) to be written into the FIFO buffer.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_temp_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set temperature FIFO enabled value.
 *
 * @param[in] enabled New temperature FIFO enabled value.
 */
void mpu6050_set_temp_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get gyroscope X-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
 * 68) to be written into the FIFO buffer.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_x_gyro_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set gyroscope X-axis FIFO enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New gyroscope X-axis FIFO enabled value.
 */
void mpu6050_set_x_gyro_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get gyroscope Y-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
 * 70) to be written into the FIFO buffer.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_y_gyro_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set gyroscope Y-axis FIFO enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New gyroscope Y-axis FIFO enabled value.
 */
void mpu6050_set_y_gyro_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get gyroscope Z-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
 * 72) to be written into the FIFO buffer.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_z_gyro_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set gyroscope Z-axis FIFO enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New gyroscope Z-axis FIFO enabled value.
 */
void mpu6050_set_z_gyro_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
 * written into the FIFO buffer.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_accel_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set accelerometer FIFO enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New accelerometer FIFO enabled value.
 */
void mpu6050_set_accel_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Slave 2 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 2 to be written into the FIFO buffer.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_2_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Slave 2 FIFO enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New Slave 2 FIFO enabled value.
 */
void mpu6050_set_slave_2_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Slave 1 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 1 to be written into the FIFO buffer.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_1_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Slave 1 FIFO enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New Slave 1 FIFO enabled value.
 */
void mpu6050_set_slave_1_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Slave 0 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 0 to be written into the FIFO buffer.
 *
 * @return Current Slave 0 FIFO enabled value.
 */
esp_err_t mpu6050_get_slave_0_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Slave 0 FIFO enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New Slave 0 FIFO enabled value.
 */
void mpu6050_set_slave_0_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get multi-master enabled value.
 * Multi-master capability allows multiple I2C masters to operate on the same
 * bus. In circuits where multi-master capability is required, set MULT_MST_EN
 * to 1. This will increase current drawn by approximately 30uA.
 *
 * In circuits where multi-master capability is required, the state of the I2C
 * bus must always be monitored by each separate I2C Master. Before an I2C
 * Master can assume arbitration of the bus, it must first confirm that no other
 * I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
 * MPU-60X0's bus arbitration detection logic is turned on, enabling it to
 * detect when the bus is available.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_multi_master_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set multi-master enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New multi-master enabled value.
 */
void mpu6050_set_multi_master_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get wait-for-external-sensor-data enabled value.
 * When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
 * delayed until External Sensor data from the Slave Devices are loaded into the
 * EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
 * data (i.e. from gyro and accel) and external sensor data have been loaded to
 * their respective data registers (i.e. the data is synced) when the Data Ready
 * interrupt is triggered.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_wait_for_external_sensor_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set wait-for-external-sensor-data enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New wait-for-external-sensor-data enabled value.
 */
void mpu6050_set_wait_for_external_sensor_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Slave 3 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 3 to be written into the FIFO buffer.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_3_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Slave 3 FIFO enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New Slave 3 FIFO enabled value.
 */
void mpu6050_set_slave_3_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get slave read/write transition enabled value.
 * The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave
 * read to the next slave read. If the bit equals 0, there will be a restart
 * between reads. If the bit equals 1, there will be a stop followed by a start
 * of the following read. When a write transaction follows a read transaction,
 * the stop followed by a start of the successive write will be always used.
 *
 * @return 
 *      - ESP_OK: enable bit equals 1
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_read_write_transition_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set slave read/write transition enabled value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New slave read/write transition enabled value.
 */
void mpu6050_set_slave_read_write_transition_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get I2C master clock speed.
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 *
 * @return Current I2C master clock speed.
 */
uint8_t mpu6050_get_master_clock_speed(const mpu6050_config_t *setting);

/**
 * @brief Set I2C master clock speed.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] speed Current I2C master clock speed.
 */
void mpu6050_set_master_clock_speed(const mpu6050_config_t *setting, uint8_t speed);

/**
 * @brief Get the I2C address of the specified slave (0-3).
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * In read mode, the result of the read is placed in the lowest available
 * EXT_SENS_DATA register. For further information regarding the allocation of
 * read results, please refer to the EXT_SENS_DATA register description
 * (Registers 73 - 96).
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions (getSlave4* and setSlave4*).
 *
 * I2C data transactions are performed at the Sample Rate, as defined in
 * Register 25. The user is responsible for ensuring that I2C data transactions
 * to and from each enabled Slave can be completed within a single period of the
 * Sample Rate.
 *
 * The I2C slave access rate can be reduced relative to the Sample Rate. This
 * reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
 * slave's access rate is reduced relative to the Sample Rate is determined by
 * I2C_MST_DELAY_CTRL (Register 103).
 *
 * The processing order for the slaves is fixed. The sequence followed for
 * processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a
 * particular Slave is disabled it will be skipped.
 *
 * Each slave can either be accessed at the sample rate or at a reduced sample
 * rate. In a case where some slaves are accessed at the Sample Rate and some
 * slaves are accessed at the reduced rate, the sequence of accessing the slaves
 * (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will
 * be skipped if their access rate dictates that they should not be accessed
 * during that particular cycle. For further information regarding the reduced
 * access rate, please refer to Register 52. Whether a slave is accessed at the
 * Sample Rate or at the reduced rate is determined by the Delay Enable bits in
 * Register 103.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 *
 * @return Current address for specified slave.
 */
uint8_t mpu6050_get_slave_address(const mpu6050_config_t *setting, uint8_t num);

/**
 * @brief Set the I2C address of the specified slave (0-3).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3)
 * @param[in] address New address for specified slave.
 */
void mpu6050_set_slave_address(const mpu6050_config_t *setting, uint8_t num, uint8_t address);

/**
 * @brief Get the active internal register for the specified slave (0-3).
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 *
 * @return Current active register for specified slave.
 */
uint8_t mpu6050_get_slave_register(const mpu6050_config_t *setting, uint8_t num);

/**
 * @brief Set the active internal register for the specified slave (0-3).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 * @param[in] reg New active register for specified slave.
 */
void mpu6050_set_slave_register(const mpu6050_config_t *setting, uint8_t num, uint8_t reg);

/**
 * @brief Get the enabled value for the specified slave (0-3).
 * When set to 1, this bit enables Slave 0 for data transfer operations. When
 * cleared to 0, this bit disables Slave 0 from data transfer operations.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_enabled(const mpu6050_config_t *setting, uint8_t num);

/**
 * @brief Set the enabled value for the specified slave (0-3).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 * @param[in] enabled New enabled value for specified slave.
 */
void mpu6050_set_slave_enabled(const mpu6050_config_t *setting, uint8_t num, bool enabled);

/**
 * @brief Get word pair byte-swapping enabled for the specified slave (0-3).
 * When set to 1, this bit enables byte swapping. When byte swapping is enabled,
 * the high and low bytes of a word pair are swapped. Please refer to
 * I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
 * bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
 * registers in the order they were transferred.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_word_byte_swap(const mpu6050_config_t *setting, uint8_t num);

/**
 * @brief Set word pair byte-swapping enabled for the specified slave (0-3).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 * @param[in] enabled New word pair byte-swapping enabled value for specified
 * slave.
 */
void mpu6050_set_slave_word_byte_swap(const mpu6050_config_t *setting, uint8_t num, bool enabled);

/**
 * @brief Get write mode for the specified slave (0-3).
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 *
 * @return Current write mode for specified slave (0 = register address + data,
 * 1 = data only).
 */
uint8_t mpu6050_get_slave_write_mode(const mpu6050_config_t *setting, uint8_t num);

/**
 * @brief Set write mode for the specified slave (0-3).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 * @param[in] mode New write mode for specified slave (0 = register address + data,
 * 1 = data only).
 */
void mpu6050_set_slave_write_mode(const mpu6050_config_t *setting, uint8_t num, bool mode);

/**
 * @brief Get word pair grouping order offset for the specified slave (0-3).
 * This sets specifies the grouping order of word pairs received from registers.
 * When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
 * then odd register addresses) are paired to form a word. When set to 1, bytes
 * from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
 * register addresses) are paired to form a word.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_word_group_offset(const mpu6050_config_t *setting, uint8_t num);

/**
 * @brief Set word pair grouping order offset for the specified slave (0-3).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 * @param[in] enabled New word pair grouping order offset for specified slave.
 */
void mpu6050_set_slave_word_group_offset(const mpu6050_config_t *setting, uint8_t num, bool enabled);

/**
 * @brief Get number of bytes to read for the specified slave (0-3).
 * Specifies the number of bytes transferred to and from Slave 0. Clearing this
 * bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 *
 * @return Number of bytes to read for specified slave.
 */
uint8_t mpu6050_get_slave_data_length(const mpu6050_config_t *setting, uint8_t num);

/**
 * @brief Set number of bytes to read for the specified slave (0-3).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 * @param[in] length Number of bytes to read for specified slave.
 */
void mpu6050_set_slave_data_length(const mpu6050_config_t *setting, uint8_t num, uint8_t length);

/**
 * @brief Get the I2C address of Slave 4.
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current address for Slave 4.
 */
uint8_t mpu6050_get_slave_4_address(const mpu6050_config_t *setting);

/**
 * @brief Set the I2C address of Slave 4.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] address New address for Slave 4.
 */
void mpu6050_set_slave_4_address(const mpu6050_config_t *setting, uint8_t address);

/**
 * @brief Get the active internal register for the Slave 4.
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * @return Current active register for Slave.
 */
uint8_t mpu6050_get_slave_4_register(const mpu6050_config_t *setting);

/**
 * @brief Set the active internal register for Slave 4.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] reg New active register for Slave 4.
 */
void mpu6050_set_slave_4_register(const mpu6050_config_t *setting, uint8_t reg);

/**
 * @brief Set new byte to write to Slave 4.
 * This register stores the data to be written into the Slave 4. If I2C_SLV4_RW
 * is set 1 (set to read), this register has no effect.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] data New byte to write to Slave 4.
 */
void mpu6050_set_slave_4_output_byte(const mpu6050_config_t *setting, uint8_t data);

/**
 * @brief Get the enabled value for the Slave 4.
 * When set to 1, this bit enables Slave 4 for data transfer operations.
 * When cleared to 0, this bit disables Slave 4 from data transfer operations.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_4_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set the enabled value for Slave 4.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New enabled value for Slave 4.
 */
void mpu6050_set_slave_4_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get the enabled value for Slave 4 transaction interrupts.
 * When set to 1, this bit enables the generation of an interrupt signal upon
 * completion of a Slave 4 transaction. When cleared to 0, this bit disables the
 * generation of an interrupt signal upon completion of a Slave 4 transaction.
 * The interrupt status can be observed in Register 54.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_4_interrupt_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set the enabled value for Slave 4 transaction interrupts.

 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New enabled value for Slave 4 transaction interrupts.
 */
void mpu6050_set_slave_4_interrupt_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get write mode for Slave 4.
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @return Current write mode for Slave 4:
 * (0 = register address + data, 1 = data only).
 */
uint8_t mpu6050_get_slave_4_write_mode(const mpu6050_config_t *setting);

/**
 * @brief Set write mode for the Slave 4.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] mode New write mode for Slave 4:
 * (0 = register address + data, 1 = data only).
 */
void mpu6050_set_slave_4_write_mode(const mpu6050_config_t *setting, bool mode);

/**
 * @brief Get Slave 4 master delay value.
 * This configures the reduced access rate of I2C slaves relative to the Sample
 * Rate. When a slave's access rate is decreased relative to the Sample Rate,
 * the slave is accessed every:
 *
 * 1 / (1 + I2C_MST_DLY) samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (Register 25) and
 * DLPF_CFG (Register 26). Whether a slave's access rate is reduced relative to
 * the Sample Rate is determined by I2C_MST_DELAY_CTRL (Register 103).
 *
 * @return Current Slave 4 master delay value.
 */
uint8_t mpu6050_get_slave_4_master_delay(const mpu6050_config_t *setting);

/**
 * @brief Set Slave 4 master delay value.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] delay New Slave 4 master delay value.
 */
void mpu6050_set_slave_4_master_delay(const mpu6050_config_t *setting, uint8_t delay);

/**
 * @brief Get last available byte read from Slave 4.
 * This register stores the data read from Slave 4. This field is populated
 * after a read transaction.
 *
 * @return Last available byte read from to Slave 4.
 */
uint8_t mpu6050_get_slave_4_input_byte(const mpu6050_config_t *setting);

/**
 * @brief Get FSYNC interrupt status.
 * This bit reflects the status of the FSYNC interrupt from an external device
 * into the MPU-60X0. This is used as a way to pass an external interrupt
 * through the MPU-60X0 to the host application processor. When set to 1, this
 * bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
 * (Register 55).
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_passthrough_status(const mpu6050_config_t *setting);

/**
 * @brief Get Slave 4 transaction done status.
 * Automatically sets to 1 when a Slave 4 transaction has completed. This
 * triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
 * (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
 * I2C_SLV4_CTRL register (Register 52).
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_4_is_done(const mpu6050_config_t *setting);

/**
 * @brief Get master arbitration lost status.
 * This bit automatically sets to 1 when the I2C Master has lost arbitration of
 * the auxiliary I2C bus (an error condition). This triggers an interrupt if the
 * I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_lost_arbitration(const mpu6050_config_t *setting);

/**
 * @brief Get Slave 4 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_4_nack(const mpu6050_config_t *setting);

/**
 * @brief Get Slave 3 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_3_nack(const mpu6050_config_t *setting);

/**
 * @brief Get Slave 2 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_2_nack(const mpu6050_config_t *setting);

/**
 * @brief Get Slave 1 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return Slave 1 NACK interrupt status.
 */
esp_err_t mpu6050_get_slave_1_nack(const mpu6050_config_t *setting);

/**
 * @brief Get Slave 0 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 *
 * @return Slave 0 NACK interrupt status.
 */
esp_err_t mpu6050_get_slave_0_nack(const mpu6050_config_t *setting);

/**
 * @brief Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * Current interrupt mode (0 = active-high, 1 = active-low).
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      ESP_OK: active-high.
 *      ESP_FAIL: active-low
 */
esp_err_t mpu6050_get_interrupt_mode(const mpu6050_config_t *setting);

/**
 * @brief Set interrupt logic level mode.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] mode New interrupt mode (0 = active-high, 1 = active-low).
 */
void mpu6050_set_interrupt_mode(const mpu6050_config_t *setting, bool mode);

/**
 * @brief Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 *
 * @return Current interrupt drive mode (0 = push-pull, 1 = open-drain).
 */
esp_err_t mpu6050_get_interrupt_drive(const mpu6050_config_t *setting);

/**
 * @brief Set interrupt drive mode.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] drive New interrupt drive mode (0 = push-pull, 1 = open-drain).
 */
void mpu6050_set_interrupt_drive(const mpu6050_config_t *setting, bool drive);

/**
 * @brief Get interrupt latch mode.
 * Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
 *
 * @return Current latch mode (0 = 50us-pulse, 1 = latch-until-int-cleared).
 */
esp_err_t mpu6050_get_interrupt_latch(const mpu6050_config_t *setting);

/**
 * @brief Set interrupt latch mode.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] latch New latch mode (0 = 50us-pulse, 1 = latch-until-int-cleared).
 */
void mpu6050_set_interrupt_latch(const mpu6050_config_t *setting, bool latch);

/**
 * @brief Get interrupt latch clear mode.
 * Will be set 0 for status-read-only, 1 for any-register-read.
 *
 * @return Current latch clear mode (0 = status-read-only,
 * 1 = any-register-read).
 */
esp_err_t mpu6050_get_interrupt_latch_clear(const mpu6050_config_t *setting);

/**
 * @brief Set interrupt latch clear mode.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] clear New latch clear mode (0 = status-read-only,
 * 1 = any-register-read).
 */
void mpu6050_set_interrupt_latch_clear(const mpu6050_config_t *setting, bool clear);

/**
 * @brief Get FSYNC interrupt logic level mode.
 *
 * @return Current FSYNC interrupt mode (0 = active-high, 1 = active-low).
 */
esp_err_t mpu6050_get_fsync_interrupt_level(const mpu6050_config_t *setting);

/**
 * @brief Set FSYNC interrupt logic level mode.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] mode New FSYNC interrupt mode (0 = active-high, 1 = active-low).
 */
void mpu6050_set_fsync_interrupt_level(const mpu6050_config_t *setting, bool level);

/**
 * @brief Get FSYNC pin interrupt enabled setting.
 * Will be set 0 for disabled, 1 for enabled.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] setting: mpu6050 condiguration
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 */
esp_err_t mpu6050_get_fsync_interrupt_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set FSYNC pin interrupt enabled setting.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New FSYNC pin interrupt enabled setting.
 */
void mpu6050_set_fsync_interrupt_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit [5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit [5]).
 *
 * @return Current I2C bypass enabled status.
 */
esp_err_t mpu6050_get_i2c_bypass_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit [5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit [5]).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New I2C bypass enabled status.
 */
void mpu6050_set_i2c_bypass_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled.
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 */
esp_err_t mpu6050_get_clock_output_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New reference clock output enabled status.
 */
void mpu6050_set_clock_output_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit will be
 * set 0 for disabled, 1 for enabled.
 * 
 * @param[in] setting: mpu6050 configuration
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 */
uint8_t mpu6050_get_int_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New interrupt enabled status.
 */
void mpu6050_set_int_enabled(const mpu6050_config_t *setting, uint8_t enabled);

/**
 * @brief Get Free Fall interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * 
 * @param[in] setting: mpu6050 condiguration
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 *      ESP_FAIL: for disabled
 */
esp_err_t mpu6050_get_int_freefall_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Free Fall interrupt enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New interrupt enabled status.
 */
void mpu6050_set_int_freefall_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 */
esp_err_t mpu6050_get_int_motion_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Motion Detection interrupt enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New interrupt enabled status.
 */
void mpu6050_set_int_motion_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Zero Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 *
 * 
 * @param[in] setting: mpu6050 condiguration
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 */
esp_err_t mpu6050_get_int_zero_motion_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Zero Motion Detection interrupt enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New interrupt enabled status.
 */
void mpu6050_set_int_zero_motion_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get FIFO Buffer Overflow interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 *
 * @param[in] setting: mpu6050 condiguration
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 */
esp_err_t mpu6050_get_int_fifo_buffer_overflow_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set FIFO Buffer Overflow interrupt enabled status.
 *
 * @param[in] enabled New interrupt enabled status.
 */
void mpu6050_set_int_fifo_buffer_overflow_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get I2C Master interrupt enabled status.
 * This enables any of the I2C Master interrupt sources to generate an
 * interrupt. Will be set 0 for disabled, 1 for enabled.
 * 
 * @param[in] setting: mpu6050 condiguration
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 */
esp_err_t mpu6050_get_int_i2c_master_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set I2C Master interrupt enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New interrupt enabled status.
 */
void mpu6050_set_int_i2c_master_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] setting: mpu6050 condiguration
 * @return 
 *      ESP_OK: for enabled.
 *      ESP_FAIL: for disabled
 */
esp_err_t mpu6050_get_int_data_ready_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Data Ready interrupt enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New interrupt enabled status.
 */
void mpu6050_set_int_data_ready_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * 
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_int_status(const mpu6050_config_t *setting);

/**
 * @brief Get Free Fall interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_int_freefall_status(const mpu6050_config_t *setting);

/**
 * @brief Get Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Motion Detection interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_int_motion_status(const mpu6050_config_t *setting);

/**
 * @brief Get Zero Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Zero Motion Detection interrupt has
 * been generated. The bit clears to 0 after the register has been read.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_int_zero_motion_status(const mpu6050_config_t *setting);

/**
 * @brief Get FIFO Buffer Overflow interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_int_fifo_buffer_overflow_status(const mpu6050_config_t *setting);

/**
 * @brief Get I2C Master interrupt status.
 * This bit automatically sets to 1 when an I2C Master interrupt has been
 * generated. For a list of I2C Master interrupts, please refer to Register 54.
 * The bit clears to 0 after the register has been read.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_int_i2c_master_status(const mpu6050_config_t *setting);

/**
 * @brief Get Data Ready interrupt status.
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_int_data_ready_status(const mpu6050_config_t *setting);

/**
 * @brief Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_XOUT is shown in the table below:
 *
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] data pointer to acceleration struct.
 */
void mpu6050_get_acceleration(const mpu6050_config_t *setting, mpu6050_acceleration_t *data);

/**
 * @brief Get X-axis accelerometer reading.
 *
 * @return X-axis acceleration measurement in 16-bit 2's complement format.
 */
int16_t mpu6050_get_acceleration_x(const mpu6050_config_t *setting);

/**
 * @brief Get Y-axis accelerometer reading.
 *
 * @return Y-axis acceleration measurement in 16-bit 2's complement format.
 */
int16_t mpu6050_get_acceleration_y(const mpu6050_config_t *setting);

/**
 * @brief Get Z-axis accelerometer reading.
 *
 * @return Z-axis acceleration measurement in 16-bit 2's complement format.
 */
int16_t mpu6050_get_acceleration_z(const mpu6050_config_t *setting);

/**
 * @brief Get current internal temperature.
 *
 * @return Temperature reading.
 */
int16_t mpu6050_get_temperature(const mpu6050_config_t *setting);

/**
 * @brief Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] data pointer to rotation struct.
 */
void mpu6050_get_rotation(const mpu6050_config_t *setting, mpu6050_rotation_t *data);

/**
 * @brief Get X-axis gyroscope reading.
 *
 * @return X-axis rotation measurement in 16-bit 2's complement format.
 */
int16_t mpu6050_get_rotation_x(const mpu6050_config_t *setting);

/**
 * @brief Get Y-axis gyroscope reading.
 *
 * @return Y-axis rotation measurement in 16-bit 2's complement format.
 */
int16_t mpu6050_get_rotation_y(const mpu6050_config_t *setting);

/**
 * @brief Get Z-axis gyroscope reading.
 *
 * @return Z-axis rotation measurement in 16-bit 2's complement format.
 */
int16_t mpu6050_get_rotation_z(const mpu6050_config_t *setting);

/**
 * @brief Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[out] data_accel pointer to acceleration struct.
 * @param[out] data_gyro pointer to rotation struct.
 */
void mpu6050_get_motion(const mpu6050_config_t *setting, mpu6050_acceleration_t *data_accel, mpu6050_rotation_t *data_gyro);

/**
 * @brief Read single byte from external sensor data register.
 * These registers store data read from external sensors by the Slave 0, 1, 2,
 * and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
 * I2C_SLV4_DI (Register 53).
 *
 * External sensor data is written to these registers at the Sample Rate as
 * defined in Register 25. This access rate can be reduced by using the Slave
 * Delay Enable registers (Register 103).
 *
 * External sensor data registers, along with the gyroscope measurement
 * registers, accelerometer measurement registers, and temperature measurement
 * registers, are composed of two sets of registers: an internal register set
 * and a user-facing read register set.
 *
 * The data within the external sensors' internal register set is always updated
 * at the Sample Rate (or the reduced access rate) whenever the serial interface
 * is idle. This guarantees that a burst read of sensor registers will read
 * measurements from the same sampling instant. Note that if burst reads are not
 * used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Data is placed in these external sensor data registers according to
 * I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39,
 * 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from
 * an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as
 * defined in Register 25) or delayed rate (if specified in Register 52 and
 * 103). During each Sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the
 * order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * Each enabled slave will have EXT_SENS_DATA registers associated with it by
 * number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from
 * EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may
 * change the higher numbered slaves' associated registers. Furthermore, if
 * fewer total bytes are being read from the external sensors as a result of
 * such a change, then the data remaining in the registers which no longer have
 * an associated slave device (i.e. high numbered registers) will remain in
 * these previously allocated registers unless reset.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of
 * available EXT_SENS_DATA registers, the excess bytes will be dropped. There
 * are 24 EXT_SENS_DATA registers and hence the total read lengths between all
 * the slaves cannot be greater than 24 or some bytes will be lost.
 *
 * Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further
 * information regarding the characteristics of Slave 4, please refer to
 * Registers 49 to 53.
 *
 * EXAMPLE:
 * Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and
 * I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that
 * I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00
 * through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05
 * will be associated with Slave 1. If Slave 2 is enabled as well, registers
 * starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
 *
 * If Slave 2 is disabled while Slave 3 is enabled in this same situation, then
 * registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3
 * instead.
 *
 * REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE:
 * If a slave is disabled at any time, the space initially allocated to the
 * slave in the EXT_SENS_DATA register, will remain associated with that slave.
 * This is to avoid dynamic adjustment of the register allocation.
 *
 * The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all
 * slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).
 *
 * This above is also true if one of the slaves gets NACKed and stops
 * functioning.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] position Starting position (0-23).
 *
 * @return Byte read from register.
 */
uint8_t mpu6050_get_external_sensor_byte(const mpu6050_config_t *setting, int position);

/**
 * @brief Read word (2 bytes) from external sensor data registers.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] position Starting position (0-21).
 *
 * @return Word read from register.
 */
uint16_t mpu6050_get_external_sensor_word(const mpu6050_config_t *setting, int position);

/**
 * @brief Read double word (4 bytes) from external sensor data registers.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] position Starting position (0-20).
 *
 * @return Double word read from registers.
 */
uint32_t mpu6050_get_external_sensor_dword(const mpu6050_config_t *setting, int position);

/**
 * @brief Get full motion detection status register content (all bits).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return Motion detection status byte.
 */
uint8_t mpu6050_get_motion_status(const mpu6050_config_t *setting);

/**
 * @brief Get X-axis negative motion detection interrupt status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable.
 */
esp_err_t mpu6050_get_x_negative_motion_detected(const mpu6050_config_t *setting);

/**
 * @brief Get X-axis positive motion detection interrupt status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_x_positive_motion_detected(const mpu6050_config_t *setting);

/**
 * @brief Get Y-axis negative motion detection interrupt status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_y_negative_motion_detected(const mpu6050_config_t *setting);

/**
 * @brief Get Y-axis positive motion detection interrupt status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_y_positive_motion_detected(const mpu6050_config_t *setting);

/**
 * @brief Get Z-axis negative motion detection interrupt status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_z_negative_motion_detected(const mpu6050_config_t *setting);

/**
 * @brief Get Z-axis positive motion detection interrupt status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_z_positive_motion_detected(const mpu6050_config_t *setting);

/**
 * @brief Get zero motion detection interrupt status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_zero_motion_detected(const mpu6050_config_t *setting);

/**
 * @brief Write byte to Data Output container for specified slave.
 * This register holds the output data written into Slave when Slave is set to
 * write mode. For further information regarding Slave control, please
 * refer to Registers 37 to 39 and immediately following.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-3).
 * @param[in] data Byte to write.
 */
void mpu6050_set_slave_output_byte(const mpu6050_config_t *setting, uint8_t num, uint8_t data);

/**
 * @brief Get external data shadow delay enabled status.
 * This register is used to specify the timing of external sensor data
 * shadowing. When DELAY_ES_SHADOW is set to 1, shadowing of external
 * sensor data is delayed until all data has been received.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_external_shadow_delay_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set external data shadow delay enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New external data shadow delay enabled status.
 */
void mpu6050_set_external_shadow_delay_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get slave delay enabled status.
 * When a particular slave delay is enabled, the rate of access for the that
 * slave device is reduced. When a slave's access rate is decreased relative to
 * the Sample Rate, the slave is accessed every:
 *
 * 1 / (1 + I2C_MST_DLY) Samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (Register * 25)
 * and DLPF_CFG (Register 26).
 *
 * For further information regarding I2C_MST_DLY, please refer to register 52.
 * For further information regarding the Sample Rate, please refer to
 * register 25.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-4).
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_slave_delay_enabled(const mpu6050_config_t *setting, uint8_t num);

/**
 * @brief Set slave delay enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] num Slave number (0-4).
 * @param[in] enabled New slave delay enabled status.
 */
void mpu6050_set_slave_delay_enabled(const mpu6050_config_t *setting, uint8_t num, bool enabled);

/**
 * @brief Reset gyroscope signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 */
void mpu6050_reset_gyroscope_path(const mpu6050_config_t *setting);

/**
 * @brief Reset accelerometer signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 */
void mpu6050_reset_accelerometer_path(const mpu6050_config_t *setting);

/**
 * @brief Reset temperature sensor signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 */
void mpu6050_reset_temperature_path(const mpu6050_config_t *setting);

/**
 * @brief Get accelerometer power-on delay.
 * The accelerometer data path provides samples to the sensor registers, Motion
 * detection, Zero Motion detection, and Free Fall detection modules. The
 * signal path contains filters which must be flushed on wake-up with new
 * samples before the detection modules begin operations. The default wake-up
 * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
 * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
 * any value above zero unless instructed otherwise by InvenSense. Please refer
 * to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
 * further information regarding the detection modules.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * 
 * @return Current accelerometer power-on delay.
 */
uint8_t mpu6050_get_accelerometer_power_on_delay(const mpu6050_config_t *setting);

/**
 * @brief Set accelerometer power-on delay.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] delay New accelerometer power-on delay (0-3).
 */
void mpu6050_set_accelerometer_power_on_delay(const mpu6050_config_t *setting, uint8_t delay);

/**
 * @brief Get Free Fall detection counter decrement configuration.
 * Detection is registered by the Free Fall detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring FF_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * FF_COUNT | Counter Decrement
 * ---------+------------------
 * 0        | Reset
 * 1        | 1
 * 2        | 2
 * 3        | 4
 *
 * When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Free Fall detection,
 * please refer to Registers 29 to 32.
 *
 * @return Current decrement configuration.
 */
uint8_t mpu6050_get_freefall_detection_counter_decrement(const mpu6050_config_t *setting);

/**
 * @brief Set Free Fall detection counter decrement configuration.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] decrement New decrement configuration value.
 */
void mpu6050_set_freefall_detection_counter_decrement(const mpu6050_config_t *setting, uint8_t decrement);

/**
 * @brief Get Motion detection counter decrement configuration.
 * Detection is registered by the Motion detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring MOT_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * MOT_COUNT | Counter Decrement
 * ----------+------------------
 * 0         | Reset
 * 1         | 1
 * 2         | 2
 * 3         | 4
 *
 * When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0.
 */
uint8_t mpu6050_get_motion_detection_counter_decrement(const mpu6050_config_t *setting);

/**
 * @brief Set Motion detection counter decrement configuration.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] decrement New decrement configuration value.
 */
void mpu6050_set_motion_detection_counter_decrement(const mpu6050_config_t *setting, uint8_t decrement);

/**
 * @brief Get FIFO enabled status.
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_fifo_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set FIFO enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New FIFO enabled status.
 */
void mpu6050_set_fifo_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get I2C Master Mode enabled status.
 * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
 * external sensor slave devices on the auxiliary I2C bus. When this bit is
 * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
 * driven by the primary I2C bus (SDA and SCL). This is a precondition to
 * enabling Bypass Mode.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_i2c_master_mode_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set I2C Master Mode enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New I2C Master Mode enabled status.
 */
void mpu6050_set_i2c_master_mode_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * Switch from I2C to SPI mode (MPU-6000 only).
 * If this is set, the primary SPI interface will be enabled in place of the
 * disabled primary I2C interface.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New switch SPIE Mode enabled status.
 */
void mpu6050_switch_spie_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 */
void mpu6050_reset_fifo(const mpu6050_config_t *setting);

/**
 * @brief Reset all sensor registers and signal paths.
 * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
 * accelerometers, and temperature sensor). This operation will also clear the
 * sensor registers. This bit automatically clears to 0 after the reset has been
 * triggered.
 *
 * When resetting only the signal path (and not the sensor registers), please
 * use Register 104, SIGNAL_PATH_RESET.
 */
void mpu6050_reset_sensors(const mpu6050_config_t *setting);

/**
 * @brief Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 */
void mpu6050_reset(const mpu6050_config_t *setting);

/**
 * @brief Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_sleep_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set sleep mode status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New sleep mode enabled status.
 */
void mpu6050_set_sleep_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get wake cycle enabled status.
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (Register 108).
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_wake_cycle_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set wake cycle enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New sleep mode enabled status.
 */
void mpu6050_set_wake_cycle_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get temperature sensor enabled status.
 * Control the usage of the internal temperature sensor.
 *
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_temp_sensor_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New temperature sensor enabled status.
 */
void mpu6050_set_temp_sensor_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get clock source setting.
 *
 * @return Current clock source setting.
 */
uint8_t mpu6050_get_clock_source(const mpu6050_config_t *setting);

/**
 * @brief Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the
 * following table:
 *
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] source New clock source setting.
 */
void mpu6050_set_clock_source(const mpu6050_config_t *setting, uint8_t source);

/**
 * @brief Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 5 Hz
 * 3            | 10 Hz
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *
 * @return Current wake frequency.
 */
uint8_t mpu6050_get_wake_frequency(const mpu6050_config_t *setting);

/**
 * @brief Set wake frequency in Accel-Only Low Power Mode.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] frequency New wake frequency.
 */
void mpu6050_set_wake_frequency(const mpu6050_config_t *setting, uint8_t frequency);

/**
 * @brief Get X-axis accelerometer standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 *
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_standby_x_accel_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set X-axis accelerometer standby enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New X-axis standby enabled status.
 */
void mpu6050_set_standby_x_accel_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Y-axis accelerometer standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_standby_y_accel_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Y-axis accelerometer standby enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New Y-axis standby enabled status.
 */
void mpu6050_set_standby_y_accel_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Z-axis accelerometer standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_standby_z_accel_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Z-axis accelerometer standby enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New Z-axis standby enabled status.
 */
void mpu6050_set_standby_z_accel_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get X-axis gyroscope standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_standby_x_gyro_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set X-axis gyroscope standby enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New X-axis standby enabled status.
 */
void mpu6050_set_standby_x_gyro_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Y-axis gyroscope standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @return 
 *      - ESP_OK: enable
 *      - ESP_FAIL: disable
 */
esp_err_t mpu6050_get_standby_y_gyro_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Y-axis gyroscope standby enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New Y-axis standby enabled status.
 */
void mpu6050_set_standby_y_gyro_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get Z-axis gyroscope standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 *
 * @param[in] setting: pointer to mpu6050 setting
 * 
 * @return Current Z-axis standby enabled status.
 */
esp_err_t mpu6050_get_standby_z_gyro_enabled(const mpu6050_config_t *setting);

/**
 * @brief Set Z-axis gyroscop]e standby enabled status.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] enabled New Z-axis standby enabled status.
 */
void mpu6050_set_standby_z_gyro_enabled(const mpu6050_config_t *setting, bool enabled);

/**
 * @brief Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (Register 35 and 36).
 *
 * 
 * @param[in] setting: pointer to mpu6050 setting
 * @return Current FIFO buffer size.
 */
uint16_t mpu6050_get_fifo_count(const mpu6050_config_t *setting);

/**
 * @brief Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * 
 * @return Byte from FIFO buffer.
 */
uint8_t mpu6050_get_fifo_byte(const mpu6050_config_t *setting);
void mpu6050_get_fifo_bytes(const mpu6050_config_t *setting, uint8_t *data, uint8_t length);

/**
 * @brief Write byte to FIFO buffer.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] data New FIFO byte of data.
 */
void mpu6050_set_fifo_byte(const mpu6050_config_t *setting, uint8_t data);

/**
 * @brief Get the identity of the device that is stored in the WHO_AM_I
 * register. The device ID is 6 bits (Should be 0x34).
 * 
 * @param[in] setting: pointer to mpu6050 setting
 */
uint8_t mpu6050_get_device_id(const mpu6050_config_t *setting);

/**
 * @brief Set a new ID into the WHO_AM_I register.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] id New identity of the device.
 */
void mpu6050_set_device_id(const mpu6050_config_t *setting, uint8_t id);

/**
 * Undocumented/DMP Registers/Functions
 */
uint8_t mpu6050_get_otp_bank_valid(const mpu6050_config_t *setting);
void mpu6050_set_otp_bank_valid(const mpu6050_config_t *setting, int8_t enabled);
int8_t mpu6050_get_x_gyro_offset_tc(const mpu6050_config_t *setting);
void mpu6050_set_x_gyro_offset_tc(const mpu6050_config_t *setting, int8_t offset);
int8_t mpu6050_get_y_gyro_offset_tc(const mpu6050_config_t *setting);
void mpu6050_set_y_gyro_offset_tc(const mpu6050_config_t *setting, int8_t offset);
int8_t mpu6050_get_z_gyro_offset_tc(const mpu6050_config_t *setting);
void mpu6050_set_z_gyro_offset_tc(const mpu6050_config_t *setting, int8_t offset);
int8_t mpu6050_get_x_fine_gain(const mpu6050_config_t *setting);
void mpu6050_set_x_fine_gain(const mpu6050_config_t *setting, int8_t gain);
int8_t mpu6050_get_y_fine_gain(const mpu6050_config_t *setting);
void mpu6050_set_y_fine_gain(const mpu6050_config_t *setting, int8_t gain);
int8_t mpu6050_get_z_fine_gain(const mpu6050_config_t *setting);
void mpu6050_set_z_fine_gain(const mpu6050_config_t *setting, int8_t gain);
int16_t mpu6050_get_x_accel_offset(const mpu6050_config_t *setting);
void mpu6050_set_x_accel_offset(const mpu6050_config_t *setting, int16_t offset);
int16_t mpu6050_get_y_accel_offset(const mpu6050_config_t *setting);
void mpu6050_set_y_accel_offset(const mpu6050_config_t *setting, int16_t offset);
int16_t mpu6050_get_z_accel_offset(const mpu6050_config_t *setting);
void mpu6050_set_z_accel_offset(const mpu6050_config_t *setting, int16_t offset);
int16_t mpu6050_get_x_gyro_offset(const mpu6050_config_t *setting);
void mpu6050_set_x_gyro_offset(const mpu6050_config_t *setting, int16_t offset);
int16_t mpu6050_get_y_gyro_offset(const mpu6050_config_t *setting);
void mpu6050_set_y_gyro_offset(const mpu6050_config_t *setting, int16_t offset);
int16_t mpu6050_get_z_gyro_offset(const mpu6050_config_t *setting);
void mpu6050_set_z_gyro_offset(const mpu6050_config_t *setting, int16_t offset);
esp_err_t mpu6050_get_int_pll_ready_enabled(const mpu6050_config_t *setting);
void mpu6050_set_int_pll_ready_enabled(const mpu6050_config_t *setting, bool enabled);
esp_err_t mpu6050_get_int_dmp_enabled(const mpu6050_config_t *setting);
void mpu6050_set_int_dmp_enabled(const mpu6050_config_t *setting, bool enabled);
esp_err_t mpu6050_get_dmp_int_5_status(const mpu6050_config_t *setting);
esp_err_t mpu6050_get_dmp_int_4_status(const mpu6050_config_t *setting);
esp_err_t mpu6050_get_dmp_int_3_status(const mpu6050_config_t *setting);
esp_err_t mpu6050_get_dmp_int_2_status(const mpu6050_config_t *setting);
esp_err_t mpu6050_get_dmp_int_1_status(const mpu6050_config_t *setting);
esp_err_t mpu6050_get_dmp_int_0_status(const mpu6050_config_t *setting);
esp_err_t mpu6050_get_int_ppl_ready_status(const mpu6050_config_t *setting);
esp_err_t mpu6050_get_int_dmp_status(const mpu6050_config_t *setting);
esp_err_t mpu6050_get_dmp_enabled(const mpu6050_config_t *setting);
void mpu6050_set_dmp_enabled(const mpu6050_config_t *setting, bool enabled);
void mpu6050_reset_dmp(const mpu6050_config_t *setting);
uint8_t mpu6050_get_dmp_config_1(const mpu6050_config_t *setting);
void mpu6050_set_dmp_config_1(const mpu6050_config_t *setting, uint8_t config);
uint8_t mpu6050_get_dmp_config_2(const mpu6050_config_t *setting);
void mpu6050_set_dmp_config_2(const mpu6050_config_t *setting, uint8_t config);

/**
 * @brief Calculates acceleration resolution.
 *
 * @param[in] accel_scale Acceleration scale. The scale range values are described
 * below:
 *
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 *
 * @return Resolution of the acceleration.
 */
float mpu6050_get_accel_res(uint8_t accel_scale);

/**
 * @brief Calculates rotation resolution.
 *
 * @param[in] accel_scale Rotation scale. The scale range values are described
 * below:
 *
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 *
 * 
 * @return Resolution of the acceleration.
 */
float mpu6050_get_gyro_res(uint8_t gyro_scale);

/**
 * @brief Function which accumulates gyro and accelerometer data after device
 * initialization. It calculates the average of the at-rest readings and then
 * loads the resulting offsets into accelerometer and gyro bias registers.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] accel_bias_res Acceleration bias resolution.
 * @param[in] gyro_bias_res Rotation bias resolution.
 */
void mpu6050_calibrate(const mpu6050_config_t *setting, float *accel_bias_res, float *gyro_bias_res);

/**
 * @brief Accelerometer and gyroscope self test.
 * Check calibration WRT factory settings.
 *
 * @param[in] setting: pointer to mpu6050 setting
 * @param[in] destination Where the result of the self test will be stored.
 */
void mpu6050_self_test(const mpu6050_config_t *setting, float *destination);

/**
 * @brief Implementation of Sebastian Madgwick's "Efficient orientation filter
 * for inertial/magnetic sensor arrays" which fuses acceleration and rotation
 * rate to produce a quaternion-based estimate of relative device orientation,
 * which can be converted to yaw, pitch, and roll.
 * The performance of the orientation filter is at least as good as conventional
 * Kalman-based filtering algorithms but is much less computationally intensive.
 * See http://www.x-io.co.uk/category/open-source/ for more details.
 *
 * @param[in] accel_x Acceleration x-axis value.
 * @param[in] accel_y Acceleration y-axis value.
 * @param[in] accel_z Acceleration z-axis value.
 * @param[in] gyro_x Rotation x-axis value.
 * @param[in] gyro_y Rotation y-axis value.
 * @param[in] gyro_z Rotation z-axis value.
 */
void mpu6050_madgwick_quaternion_update(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y,
    float gyro_z);
#endif

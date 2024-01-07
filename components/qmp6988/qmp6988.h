
#ifndef __QMP6988_H__
#define __QMP6988_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QMP6988_U16_t uint16_t
#define QMP6988_S16_t int16_t
#define QMP6988_U32_t uint32_t
#define QMP6988_S32_t int32_t
#define QMP6988_U64_t uint64_t
#define QMP6988_S64_t int64_t

#define QMP6988_I2C_ADDR_GND 0x70
#define QMP6988_I2C_ADDR_VDD 0x56

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

/**
 * Possible measurement modes
 */
typedef enum {
    QMP6988_SLEEP_MODE = 0x00,  // sleep mode
    QMP6988_FORCED_MODE = 0x01, // one measurement then sleep again
    QMP6988_NORMAL_MODE = 0x03  // power mode
} qmp6988_power_mode_t;

#define QMP6988_CTRLMEAS_REG_MODE__POS 0
#define QMP6988_CTRLMEAS_REG_MODE__MSK 0x03
#define QMP6988_CTRLMEAS_REG_MODE__LEN 2

/**
 * Possible filter modes
 */
typedef enum {
    QMP6988_FILTERCOEFF_OFF = 0x00,
    QMP6988_FILTERCOEFF_2 = 0x01,
    QMP6988_FILTERCOEFF_4 = 0x02,
    QMP6988_FILTERCOEFF_8 = 0x03,
    QMP6988_FILTERCOEFF_16 = 0x04,
    QMP6988_FILTERCOEFF_32 = 0x05
} qmp6988_filter_t;

#define QMP6988_CONFIG_REG             0xF1 /*IIR filter co-efficient setting Register*/
#define QMP6988_CONFIG_REG_FILTER__POS 0
#define QMP6988_CONFIG_REG_FILTER__MSK 0x07
#define QMP6988_CONFIG_REG_FILTER__LEN 3

/**
 * Possible oversampling modes
 */
typedef enum {
    QMP6988_OVERSAMPLING_SKIPPED = 0x00,
    QMP6988_OVERSAMPLING_1X = 0x01,
    QMP6988_OVERSAMPLING_2X = 0x02,
    QMP6988_OVERSAMPLING_4X = 0x03,
    QMP6988_OVERSAMPLING_8X = 0x04,
    QMP6988_OVERSAMPLING_16X = 0x05,
    QMP6988_OVERSAMPLING_32X = 0x06,
    QMP6988_OVERSAMPLING_64X = 0x07
} qmp6988_oversampling_t;

#define QMP6988_CTRLMEAS_REG_OSRST__POS 5
#define QMP6988_CTRLMEAS_REG_OSRST__MSK 0xE0
#define QMP6988_CTRLMEAS_REG_OSRST__LEN 3

#define QMP6988_CTRLMEAS_REG_OSRSP__POS 2
#define QMP6988_CTRLMEAS_REG_OSRSP__MSK 0x1C
#define QMP6988_CTRLMEAS_REG_OSRSP__LEN 3

// #define QMP6988_RAW_DATA_SIZE 8
typedef uint8_t qmp6988_raw_data_t;

typedef struct _qmp6988_cali_data
{
    QMP6988_S32_t COE_a0;
    QMP6988_S16_t COE_a1;
    QMP6988_S16_t COE_a2;
    QMP6988_S32_t COE_b00;
    QMP6988_S16_t COE_bt1;
    QMP6988_S16_t COE_bt2;
    QMP6988_S16_t COE_bp1;
    QMP6988_S16_t COE_b11;
    QMP6988_S16_t COE_bp2;
    QMP6988_S16_t COE_b12;
    QMP6988_S16_t COE_b21;
    QMP6988_S16_t COE_bp3;
} qmp6988_cali_data_t;

typedef struct _qmp6988_fk_data
{
    float a0, b00;
    float a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_fk_data_t;

typedef struct _qmp6988_ik_data
{
    QMP6988_S32_t a0, b00;
    QMP6988_S32_t a1, a2;
    QMP6988_S64_t bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_ik_data_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev; //!< I2C device descriptor

    qmp6988_power_mode_t power_mode;            //!< used power mode
    qmp6988_filter_t filter_mode;               //!< used filter mode
    qmp6988_oversampling_t oversampling_t_mode; //!< used oversampling temp mode
    qmp6988_oversampling_t oversampling_p_mode; //!< used oversampling pressure mode

    qmp6988_cali_data_t qmp6988_cali;
    qmp6988_ik_data_t ik;

    float temperature;
    float pressure;

    // bool meas_started;            //!< indicates whether measurement started
    // uint64_t meas_start_time;     //!< measurement start time in us
    // bool meas_first;              //!< first measurement in periodic mode
} qmp6988_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param port      I2C port
 * @param addr      Device address
 * @param sda_gpio  SDA GPIO
 * @param scl_gpio  SCL GPIO
 * @return          `ESP_OK` on success
 */
esp_err_t qmp6988_init_desc(qmp6988_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t qmp6988_free_desc(qmp6988_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t qmp6988_init(qmp6988_t *dev);

esp_err_t qmp6988_setup_powermode(qmp6988_t *dev, qmp6988_power_mode_t power_mode);

esp_err_t qmp6988_set_filter(qmp6988_t *dev, qmp6988_filter_t filter_mode);

esp_err_t qmp6988_set_p_oversampling(qmp6988_t *dev, qmp6988_oversampling_t oversampling_p_mode);

esp_err_t qmp6988_set_t_oversampling(qmp6988_t *dev, qmp6988_oversampling_t oversampling_t_mode);

float qmp6988_calc_pressure(qmp6988_t *dev);

float qmp6988_calc_temperature(qmp6988_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __QMP6988_H__ */

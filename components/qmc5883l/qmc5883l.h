/**
 * @file qmc5883l.h
 * @defgroup qmc5883l qmc5883l
 * @{
 *
 * ESP-IDF Driver for 3-axis magnetic sensor QMC5883L
 *
 * Copyright (C) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __QMC5883L_H__
#define __QMC5883L_H__

#include <stdint.h>
#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Default I2C address
 */
#define QMC5883L_I2C_ADDR_DEF 0x0d

/**
 * Output data rate
 */
typedef enum {
    QMC5883L_DR_10 = 0, //!< 10Hz
    QMC5883L_DR_50,     //!< 50Hz
    QMC5883L_DR_100,    //!< 100Hz
    QMC5883L_DR_200,    //!< 200Hz
} qmc5883l_odr_t;

/**
 * Oversampling rate
 */
typedef enum {
    QMC5883L_OSR_64 = 0, //!< 64 samples
    QMC5883L_OSR_128,    //!< 128 samples
    QMC5883L_OSR_256,    //!< 256 samples
    QMC5883L_OSR_512,    //!< 512 samples
} qmc5883l_osr_t;

/**
 * Field range
 */
typedef enum {
    QMC5883L_RNG_2 = 0,//!< -2G..+2G
    QMC5883L_RNG_8     //!< -8G..+8G
} qmc5883l_range_t;

/**
 * Mode
 */
typedef enum {
    QMC5883L_MODE_STANDBY = 0, //!< Standby low power mode, no measurements
    QMC5883L_MODE_CONTINUOUS   //!< Continuous measurements
} qmc5883l_mode_t;


/**
 * Raw measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883l_raw_data_t;

/**
 * Measurement result, milligauss
 */
typedef struct
{
    float x;
    float y;
    float z;
} qmc5883l_data_t;

/**
 * Device descriptor
 */
typedef struct {
    i2c_dev_t i2c_dev;
    qmc5883l_range_t range;
} qmc5883l_t;

/**
 * @brief Initialize device descriptior
 * @param dev Device descriptor
 * @param port I2C port number
 * @param addr I2C address
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_init_desc(qmc5883l_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_free_desc(qmc5883l_t *dev);

/**
 * Reset device
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_reset(qmc5883l_t *dev);

/**
 * Read chip ID
 * @param dev Device descriptor
 * @param id Chip ID
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_get_chip_id(qmc5883l_t *dev, uint8_t *id);

/**
 * Set device mode
 * @param dev Device descriptor
 * @param mode Mode
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_set_mode(qmc5883l_t *dev, qmc5883l_mode_t mode);

/**
 * Read current device mode
 * @param dev Device descriptor
 * @param mode Mode
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_get_mode(qmc5883l_t *dev, qmc5883l_mode_t *mode);

/**
 * Set device configuration
 * @param dev Device descriptor
 * @param odr Output data rate
 * @param osr Oversampling
 * @param rng Field range
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_set_config(qmc5883l_t *dev, qmc5883l_odr_t odr, qmc5883l_osr_t osr, qmc5883l_range_t rng);

/**
 * Read current device configuration
 * @param dev Device descriptor
 * @param odr Output data rate
 * @param osr Oversampling
 * @param rng Field range
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_get_config(qmc5883l_t *dev, qmc5883l_odr_t *odr, qmc5883l_osr_t *osr, qmc5883l_range_t *rng);

/**
 * Enable/disable interrupt pin
 * @param dev Device descriptor
 * @param enable Enable interrupt if true
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_set_int(qmc5883l_t *dev, bool enable);

/**
 * Get interrupt pin state
 * @param dev Device descriptor
 * @param enable Interrupt pin enabled if true
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_get_int(qmc5883l_t *dev, bool *enable);

/**
 * Get magnetic data state
 * @param dev Device descriptor
 * @param ready Magnetic data ready to read if true
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_data_ready(qmc5883l_t *dev, bool *ready);

/**
 * Read raw magnetic data
 * @param dev Device descriptor
 * @param raw Raw magnetic data
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_get_raw_data(qmc5883l_t *dev, qmc5883l_raw_data_t *raw);

/**
 * Convert raw magnetic data to milligauss
 * @param dev Device descriptor
 * @param raw Raw magnetic data
 * @param data Magnetic data in mG
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_raw_to_mg(qmc5883l_t *dev, qmc5883l_raw_data_t *raw, qmc5883l_data_t *data);

/**
 * Read magnetic data in milligauss
 * @param dev Device descriptor
 * @param data Magnetic data in mG
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_get_data(qmc5883l_t *dev, qmc5883l_data_t *data);

/**
 * Read raw temperature data (see datasheet)
 * @param dev Device descriptor
 * @param temp Raw temperature data
 * @return `ESP_OK` on success
 */
esp_err_t qmc5883l_get_raw_temp(qmc5883l_t *dev, int16_t *temp);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __QMC5883L_H__ */

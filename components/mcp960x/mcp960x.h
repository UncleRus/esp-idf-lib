/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file mcp960x.h
 * @defgroup mcp960x mcp960x
 * @{
 *
 * ESP-IDF driver for MCP960X/L0X/RL0X
 * Thermocouple EMF to Temperature Converter
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP960X_H__
#define __MCP960X_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Basic I2C address. Device support 8 addresses, ranging from 0x60 to 0x67.
 */
#define MCP960X_ADDR_BASE     0x60
/**
 * Default I2C address (ADDR pin is connected to VDD or floating).
 */
#define MCP960X_ADDR_DEFAULT  0x67

#define MCP960X_FILTER_OFF 0
#define MCP960X_FILTER_MAX 7

/**
 * Thermocouple type
 */
typedef enum {
    MCP960X_TYPE_K = 0, /**< default */
    MCP960X_TYPE_J,
    MCP960X_TYPE_T,
    MCP960X_TYPE_N,
    MCP960X_TYPE_S,
    MCP960X_TYPE_E,
    MCP960X_TYPE_B,
    MCP960X_TYPE_R,
} mcp960x_thermocouple_t;

/**
 * ADC measurement resolution
 */
typedef enum {
    MCP960X_ADC_RES_18 = 0, /**< 18 bits / 2 uV, 320 ms, default */
    MCP960X_ADC_RES_16,     /**< 16 bits / 8 uV, 80 ms */
    MCP960X_ADC_RES_14,     /**< 14 bits / 32 uV, 20 ms */
    MCP960X_ADC_RES_12,     /**< 12 bits / 128 uV, 5 ms */
} mcp960x_adc_resolution_t;

/**
 * Number of temperature samples in burst mode
 */
typedef enum {
    MCP960X_SAMPLES_1 = 0, /**< default */
    MCP960X_SAMPLES_2,
    MCP960X_SAMPLES_4,
    MCP960X_SAMPLES_8,
    MCP960X_SAMPLES_16,
    MCP960X_SAMPLES_32,
    MCP960X_SAMPLES_64,
    MCP960X_SAMPLES_128,
} mcp960x_burst_samples_t;

/**
 * Operational mode
 */
typedef enum {
    MCP960X_MODE_NORMAL = 0, /**< default */
    MCP960X_MODE_SHUTDOWN,
    MCP960X_MODE_BURST,
} mcp960x_mode_t;

/**
 * Cold-Junction/Ambient sensor resolution
 */
typedef enum {
    MCP960X_TC_RES_0_0625 = 0, /**< 0.0625°C, conversion time: 250 ms */
    MCP960X_TC_RES_0_25,       /**< 0.25°C, conversion time: 63 ms */
} mcp960x_tc_resolution_t;

/**
 * Device status
 */
typedef enum {
    MCP960X_OK = 0,        /**< Device is functioning normally */
    MCP960X_OPEN_CIRCUIT,  /**< Open circuit detected on thermocouple input (MCP9601 only) */
    MCP960X_SHORT_CIRCUIT, /**< Short circuit detected on thermocouple input (MCP9601 only) */
} mcp960x_status_t;

/**
 * Alert
 */
typedef enum {
    MCP960X_ALERT_1 = 0,
    MCP960X_ALERT_2,
    MCP960X_ALERT_3,
    MCP960X_ALERT_4,
} mcp960x_alert_t;

/**
 * Alert mode
 */
typedef enum {
    MCP960X_ALERT_DISABLED, /**< Alert disabled */
    MCP960X_ALERT_COMP,     /**< Comparator mode */
    MCP960X_ALERT_INT,      /**< Interrupt mode */
} mcp960x_alert_mode_t;

/**
 * Alert output active level
 */
typedef enum {
    MCP960X_ACTIVE_LOW = 0, /**< Active-low */
    MCP960X_ACTIVE_HIGH     /**< Active-high */
} mcp960x_alert_level_t;

/**
 * Alert temperature direction
 */
typedef enum {
    MCP960X_FALLING = 0, /**< Alert limit for falling or cooling temperatures */
    MCP960X_RISING       /**< Alert limit for rising or heating temperatures */
} mcp960x_alert_temp_dir_t;

/**
 * Alert monitoring source
 */
typedef enum {
    MCP960X_ALERT_SRC_TH = 0, /**< Alert monitor for thermocouple temperature TH */
    MCP960X_ALERT_SRC_TC      /**< Alert monitor for cold-junction sensor TC */
} mcp960x_alert_source_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev; /**< I2C device descriptor */
    uint8_t id;        /**< Hardware ID */
    uint8_t revision;  /**< Hardware revision */
} mcp960x_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_init_desc(mcp960x_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_free_desc(mcp960x_t *dev);

/**
 * @brief Init device
 *
 * Read device HW ID and revision
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_init(mcp960x_t *dev);

/**
 * @brief Setup thermocouple parameters
 *
 * @param dev Device descriptor
 * @param th Thermocouple type
 * @param filter Digital filtering level, 0..MCP960X_FILTER_MAX
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_set_sensor_config(mcp960x_t *dev, mcp960x_thermocouple_t th, uint8_t filter);

/**
 * @brief Get thermocouple parameters
 *
 * @param dev Device descriptor
 * @param[out] th Thermocouple type, NULL-able
 * @param[out] filter Digital filtering level, 0..MCP960X_FILTER_MAX, NULL-able
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_sensor_config(mcp960x_t *dev, mcp960x_thermocouple_t *th, uint8_t *filter);

/**
 * @brief Set device configuration
 *
 * @param dev Device descriptor
 * @param mode Operation mode
 * @param bs Number of temperature samples in burst mode
 * @param adc_res ADC measurement resolution
 * @param tc_res Cold-Junction/Ambient sensor resolution
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_set_device_config(mcp960x_t *dev, mcp960x_mode_t mode, mcp960x_burst_samples_t bs,
        mcp960x_adc_resolution_t adc_res, mcp960x_tc_resolution_t tc_res);

/**
 * @brief Get device configuration
 *
 * @param dev Device descriptor
 * @param[out] mode Operation mode, NULL-able
 * @param[out] bs Number of temperature samples in burst mode, NULL-able
 * @param[out] adc_res ADC measurement resolution, NULL-able
 * @param[out] tc_res Cold-Junction/Ambient sensor resolution, NULL-able
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_device_config(mcp960x_t *dev, mcp960x_mode_t *mode, mcp960x_burst_samples_t *bs,
        mcp960x_adc_resolution_t *adc_res, mcp960x_tc_resolution_t *tc_res);

/**
 * @brief Switch device operation mode
 *
 * @param dev Device descriptor
 * @param mode Operation mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_set_mode(mcp960x_t *dev, mcp960x_mode_t mode);

/**
 * @brief Get raw ADC data
 *
 * @param dev Device descriptor
 * @param[out] data Raw ADC data
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_raw_adc_data(mcp960x_t *dev, int32_t *data);

/**
 * @brief Get thermocouple temperature
 *
 * Returns cold-junction compensated and error-corrected
 * thermocouple temperature
 *
 * @param dev Device descriptor
 * @param[out] t Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_thermocouple_temp(mcp960x_t *dev, float *t);

/**
 * @brief Get thermocouple junctions delta temperature
 *
 * Returns error corrected thermocouple hot-junction temperature without the
 * cold-junction compensation
 *
 * @param dev Device descriptor
 * @param[out] t Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_delta_temp(mcp960x_t *dev, float *t);

/**
 * @brief Get cold-junction/ambient temperature
 *
 * @param dev Device descriptor
 * @param[out] t Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_ambient_temp(mcp960x_t *dev, float *t);

/**
 * @brief Get device status
 *
 * @param dev Device descriptor
 * @param[out] temp_ready Temperature conversion complete if true, NULL-able
 * @param[out] burst_ready Burst conversion complete if true, NULL-able (burst mode only)
 * @param[out] status Device status, NULL-able
 * @param[out] alert1 Alert 1 status, NULL-able
 * @param[out] alert2 Alert 2 status, NULL-able
 * @param[out] alert3 Alert 3 status, NULL-able
 * @param[out] alert4 Alert 4 status, NULL-able
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_status(mcp960x_t *dev, bool *temp_ready, bool *burst_ready, mcp960x_status_t *status,
        bool *alert1, bool *alert2, bool *alert3, bool *alert4);

/**
 * @brief Setup temperature alert
 *
 * @param dev Device descriptor
 * @param alert Alert index
 * @param mode Alert mode
 * @param active_lvl Alert output active level
 * @param temp_dir Alert temperature direction
 * @param src Alert temperature source
 * @param limit Alert limit
 * @param hyst Hysteresis
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_set_alert_config(mcp960x_t *dev, mcp960x_alert_t alert, mcp960x_alert_mode_t mode,
        mcp960x_alert_level_t active_lvl, mcp960x_alert_temp_dir_t temp_dir, mcp960x_alert_source_t src,
        float limit, uint8_t hyst);

/**
 * @brief Get temperature alert configuration
 *
 * @param dev Device descriptor
 * @param alert Alert index
 * @param[out] mode Alert mode, NULL-able
 * @param[out] active_lvl Alert output active level, NULL-able
 * @param[out] temp_dir Alert temperature direction, NULL-able
 * @param[out] src Alert temperature source, NULL-able
 * @param[out] limit Alert limit, NULL-able
 * @param[out] hyst Hysteresis, NULL-able
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_alert_config(mcp960x_t *dev, mcp960x_alert_t alert, mcp960x_alert_mode_t *mode,
        mcp960x_alert_level_t *active_lvl, mcp960x_alert_temp_dir_t *temp_dir, mcp960x_alert_source_t *src,
        float *limit, uint8_t *hyst);

/**
 * @brief Get alert status
 *
 * @param dev Device descriptor
 * @param alert Alert index
 * @param[out] status Alert status
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_get_alert_status(mcp960x_t *dev, mcp960x_alert_t alert, bool *status);

/**
 * @brief Clear alert interrupt
 *
 * @param dev Device descriptor
 * @param alert Alert index
 * @return `ESP_OK` on success
 */
esp_err_t mcp960x_clear_alert_int(mcp960x_t *dev, mcp960x_alert_t alert);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MCP960X_H__ */

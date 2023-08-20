/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file bh1900nux.h
 * @defgroup bh1900nux bh1900nux
 * @{
 *
 * ESP-IDF driver for BH1900NUX temperature sensor
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __BH1900NUX_H__
#define __BH1900NUX_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#define BH1900NUX_I2C_ADDR_BASE 0x48 //!< See full list in datasheet

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fault queue size
 */
typedef enum {
    BH1900NUX_FAULTS_1 = 0, //!< 1 fault to trigger ALERT pin
    BH1900NUX_FAULTS_2,     //!< 2 faults to trigger ALERT pin
    BH1900NUX_FAULTS_4,     //!< 4 faults to trigger ALERT pin
    BH1900NUX_FAULTS_6      //!< 6 faults to trigger ALERT pin
} bh1900nux_fault_queue_t;

/**
 * ALERT pin polarity
 */
typedef enum {
    BH1900NUX_ALERT_LOW = 0, //!< ALERT active low (default)
    BH1900NUX_ALERT_HIGH     //!< ALERT active high
} bh1900nux_alert_polarity_t;

/**
 * Device operating mode
 */
typedef enum {
    BH1900NUX_MODE_CONTINUOUS = 0, //!< Continuous measurement mode (default)
    BH1900NUX_MODE_SHUTDOWN        //!< Shutdown mode
} bh1900nux_mode_t;

/**
 * Delay between measurements in continuous mode
 */
typedef enum {
    BH1900NUX_WT_0 = 0, //!< 186240 * 16 / 450000 ~ 6.622s (Fosc = 450kHz)
    BH1900NUX_WT_1,     //!< 186240 * 4 / 450000 ~ 1.655s (Fosc = 450kHz)
    BH1900NUX_WT_2,     //!< 186240 / 450000 ~ 0.414s (Fosc = 450kHz)
    BH1900NUX_WT_3,     //!< 93120 / 450000 ~ 0.207s (Fosc = 450kHz)
} bh1900nux_wait_time_t;

/**
 * Device descriptor
 */
typedef struct {
    i2c_dev_t i2c_dev;
    bh1900nux_mode_t mode;
} bh1900nux_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port number
 * @param addr I2C address
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_init_desc(bh1900nux_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_free_desc(bh1900nux_t *dev);

/**
 * @brief Software reset
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_reset(bh1900nux_t *dev);

/**
 * @brief Set device operating mode
 *
 * @param dev Device descriptor
 * @param mode Operating mode
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_set_mode(bh1900nux_t *dev, bh1900nux_mode_t mode);

/**
 * @brief Read current device config
 *
 * @param dev Device descriptor
 * @param[out] wt Delay between measurements
 * @param[out] fq Fault queue size
 * @param[out] ap ALERT pin polarity
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_get_config(bh1900nux_t *dev, bh1900nux_wait_time_t *wt, bh1900nux_fault_queue_t *fq, bh1900nux_alert_polarity_t *ap);

/**
 * @brief Configure device
 *
 * @param dev Device descriptor
 * @param wt Delay between measurements
 * @param fq Fault queue size
 * @param ap ALERT pin polarity
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_set_config(bh1900nux_t *dev, bh1900nux_wait_time_t wt, bh1900nux_fault_queue_t fq, bh1900nux_alert_polarity_t ap);

/**
 * @brief Made a single-shot measurement
 *
 * Works only when device is in shutdown mode.
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, deg.C
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_one_shot(bh1900nux_t *dev, float *temp);

/**
 * @brief Read temperature register
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, deg.C
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_get_temperature(bh1900nux_t *dev, float *temp);

/**
 * @brief Read lower temperature limit register
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, deg.C
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_get_t_low(bh1900nux_t *dev, float *temp);

/**
 * @brief Write lower temperature limit register
 *
 * @param dev Device descriptor
 * @param temp Temperature, deg.C
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_set_t_low(bh1900nux_t *dev, float temp);

/**
 * @brief Read higher temperature limit register
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, deg.C
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_get_t_high(bh1900nux_t *dev, float *temp);

/**
 * @brief Write higher temperature limit register
 *
 * @param dev Device descriptor
 * @param temp Temperature, deg.C
 * @return `ESP_OK` on success
 */
esp_err_t bh1900nux_set_t_high(bh1900nux_t *dev, float temp);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __BH1900NUX_H__ */

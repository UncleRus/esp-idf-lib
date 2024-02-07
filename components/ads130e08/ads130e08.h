/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Weslley M. F. Duarte <weslleymfd@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
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
 */

/**
 * @file ads130e08.h
 * @defgroup ads130e08 ads130e08
 * @{
 *
 * ESP-IDF driver for ADS130E08 ADC
 *
 * Copyright (c) 2021 Weslley M. F. Duarte <weslleymfd@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */

#ifndef __ADS130E08_H__
#define __ADS130E08_H__

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ADS130E08_CMD_WAKEUP = 0x02,
    ADS130E08_CMD_STANDBY = 0x04,
    ADS130E08_CMD_RESET = 0x06,
    ADS130E08_CMD_START = 0x08,
    ADS130E08_CMD_STOP = 0x0A
} ads130e08_system_cmd_t;

typedef enum {
    ADS130E08_CMD_RDATAC = 0x10,
    ADS130E08_CMD_SDATAC = 0x11,
    ADS130E08_CMD_RDATA = 0x12
} ads130e08_data_read_cmd_t;

typedef enum {
    ADS130E08_CLK_OUT_DISABLED = 0x00, /**< Oscillator clock output disabled (default) */
    ADS130E08_CLK_OUT_ENABLED = 0x20   /**< Oscillator clock output enabled */
} ads130e08_clk_en_t;

typedef enum {
    ADS130E08_INT_TEST_EXTERNAL = 0x00, /**< Test signals are driven externally (default) */
    ADS130E08_INT_TEST_INTERNAL = 0x10  /**< Test signals are generated internally */
} ads130e08_int_test_t;

typedef enum {
    ADS130E08_TEST_AMP_CALIB_1X = 0x00, /**< 1 × –(VREFP – VREFN) / 2.4 mV (default) */
    ADS130E08_TEST_AMP_CALIB_2X = 0x04  /**< 2 × –(VREFP – VREFN) / 2.4 mV */
} ads130e08_test_amp_t;

typedef enum {
    ADS130E08_TEST_FREQ_EXP_21 = 0x00, /**< Pulsed at fCLK / 2^21 (default) */
    ADS130E08_TEST_FREQ_EXP_20 = 0x01, /**< Pulsed at fCLK / 2^20 */
    ADS130E08_TEST_FREQ_AT_DC = 0x11   /**< At dc */
} ads130e08_test_freq_t;

typedef enum {
    ADS130E08_INTERNAL_REF_BUFFER_DISABLED = 0x00, /**< Power-down internal reference buffer (default) */
    ADS130E08_INTERNAL_REF_BUFFER_ENABLED = 0x80   /**< Enable internal reference buffer */
} ads130e08_pd_refbuf_t;

typedef enum {
    ADS130E08_REF_VOLTAGE_2_4V = 0x00, /**< VREFP is set to 2.4 V (default) */
    ADS130E08_REF_VOLTAGE_4_0V = 0x20, /**< VREFP is set to 4 V (only use with a 5-V analog supply) */
} ads130e08_vref_4v_t;

typedef enum {
    ADS130E08_NON_INVERTING_CONNECT_OPAMP = 0x00, /**< Noninverting input connected to the OPAMPP pin (default) */
    ADS130E08_NON_INVERTING_CONNECT_AV = 0x08,    /**< Noninverting input connected to (AVDD + AVSS) / 2 */
} ads130e08_opamp_ref_t;

typedef enum {
    ADS130E08_OPAMP_DISABLED = 0x00, /**< Power-down op amp (default) */
    ADS130E08_OPAMP_ENABLED = 0x04,  /**< Enable op amp */
} ads130e08_pd_opamp_t;

/**
 * Channels
 */
typedef enum {
    ADS130E08_CHANNEL_1 = 0x05,
    ADS130E08_CHANNEL_2 = 0x06,
    ADS130E08_CHANNEL_3 = 0x07,
    ADS130E08_CHANNEL_4 = 0x08,
    ADS130E08_CHANNEL_5 = 0x09,
    ADS130E08_CHANNEL_6 = 0x0A,
    ADS130E08_CHANNEL_7 = 0x0B,
    ADS130E08_CHANNEL_8 = 0x0C
} ads130e08_channel_t;

/**
 * Fault detect comparator threshold
 */
typedef enum {
    ADS130E08_MODE_1 = 0x00, /**< Comparator positive threshold: 95% , negative threshold: 5% (default) */
    ADS130E08_MODE_2 = 0x20, /**< Comparator positive threshold: 92.5% , negative threshold: 7.5% (default) */
    ADS130E08_MODE_3 = 0x40, /**< Comparator positive threshold: 90% , negative threshold: 10% (default) */
    ADS130E08_MODE_4 = 0x60, /**< Comparator positive threshold: 87.5% , negative threshold: 12.5% (default) */
    ADS130E08_MODE_5 = 0x80, /**< Comparator positive threshold: 85% , negative threshold: 15% (default) */
    ADS130E08_MODE_6 = 0xA0, /**< Comparator positive threshold: 80% , negative threshold: 20% (default) */
    ADS130E08_MODE_7 = 0xC0, /**< Comparator positive threshold: 75% , negative threshold: 25% (default) */
    ADS130E08_MODE_8 = 0xE0  /**< Comparator positive threshold: 70% , negative threshold: 30% (default) */
} ads130e08_fault_threshold_t;

/**
 * Power down
 */
typedef enum {
    ADS130E08_NORMAL_OPERATION = 0x00, /**< Normal operation (default) */
    ADS130E08_POWER_DOWN = 0x80        /**< Channel power-down */
} ads130e08_pd_t;

/**
 * PGA gain
 */
typedef enum {
    ADS130E08_PGA_1 = 0x10, /**< x1 */
    ADS130E08_PGA_2 = 0x20, /**< x2 */
    ADS130E08_PGA_8 = 0x50  /**< x8 */
} ads130e08_pga_gain_t;

/**
 * MUX
 */
typedef enum {
    ADS130E08_NORMAL_INPUT = 0x00,       /**< Normal input (default) */
    ADS130E08_INPUT_SHORTED = 0x01,      /**< Input shorted (for offset or noise measurements) */
    ADS130E08_MVDD = 0x03,               /**< MVDD for supply measurement */
    ADS130E08_TEMPERATURE_SENSOR = 0x04, /**< Temperature sensor */
    ADS130E08_TEST_SIGNAL = 0x05         /**< Test signal */
} ads130e08_mux_t;

typedef enum { ADS130E08_FAULT_STATP = 0x12, ADS130E08_FAULT_STATN = 0x13 } ads130e08_fault_t;

/**
 * Fault status
 */
typedef enum {
    ADS130E08_NO_FAULT_PRESENT = 0x00, /**< No fault present (default) */
    ADS130E08_FAULT_PRESENT = 0x01     /**< Fault present */
} ads130e08_fault_status_t;

/**
 * GPIO mode
 */
typedef enum {
    ADS130E08_GPIO_OUTPUT = 0x00, /**< Output */
    ADS130E08_GPIO_INPUT = 0x01   /**< Input (default) */
} ads130e08_gpio_mode_t;

/**
 * GPIO level
 */
typedef enum { ADS130E08_GPIO_RESET = 0x00, ADS130E08_GPIO_SET = 0x01 } ads130e08_gpio_level_t;

/**
 * GPIOs
 */
typedef enum {
    ADS130E08_GPIO1 = 0x01,
    ADS130E08_GPIO2 = 0x02,
    ADS130E08_GPIO3 = 0x03,
    ADS130E08_GPIO4 = 0x04
} ads130e08_gpio_pin_t;

/**
 * Number of devices
 */
typedef enum { ADS130E08_DEVICES_1 = 0x01, ADS130E08_DEVICES_2 = 0x02 } ads130e08_devices_n_t;

/**
 * Device descriptor
 */
typedef struct
{
    spi_device_interface_config_t spi_cfg; /**< SPI device configuration */
    spi_device_handle_t spi_dev;           /**< SPI device handler */
} ads130e08_t;

/**
 * Device configuration
 */
typedef struct
{
    ads130e08_clk_en_t clk_en;
    ads130e08_int_test_t int_test;
    ads130e08_test_amp_t test_amp;
    ads130e08_test_freq_t test_freq;
    ads130e08_pd_refbuf_t pd_refbuf;
    ads130e08_vref_4v_t vref_4v;
    ads130e08_opamp_ref_t opamp_ref;
    ads130e08_pd_opamp_t pd_opamp;
} ads130e08_dev_config_t;

/**
 * Channel configuration
 */
typedef struct
{
    ads130e08_pd_t enable;
    ads130e08_pga_gain_t pga_gain;
    ads130e08_mux_t mode;
} ads130e08_channel_config_t;

typedef struct
{
    uint8_t fault_statp;
    uint8_t fault_statn;
    uint8_t gpios_level;
    int16_t channels_raw[8];
} ads130e08_raw_data_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev    Device descriptor
 * @param host   SPI host
 * @param cs_pin CS GPIO number
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_init_desc(ads130e08_t *dev, spi_host_device_t host, gpio_num_t cs_pin);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_free_desc(ads130e08_t *dev);

/**
 * @brief Send system_command to device
 *
 * @param dev Device descriptor
 * @param cmd Command
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_send_system_cmd(ads130e08_t *dev, ads130e08_system_cmd_t cmd);

/**
 * @brief Send data_read_command to device
 *
 * @param dev Device descriptor
 * @param cmd Command
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_send_data_read_cmd(ads130e08_t *dev, ads130e08_data_read_cmd_t cmd);

/**
 * @brief Get device id
 *
 * @param dev Device descriptor
 * @param id  Id
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_get_device_id(ads130e08_t *dev, uint8_t *id);

/**
 * @brief Set device configuration
 *
 * @param dev    Device descriptor
 * @param config Device configurations
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_set_device_config(ads130e08_t *dev, ads130e08_dev_config_t config);

/**
 * @brief Get device configuration
 *
 * @param dev    Device descriptor
 * @param config Device configurations
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_get_device_config(ads130e08_t *dev, ads130e08_dev_config_t *config);

/**
 * @brief Reads raw data in "Read data by command" mode
 *
 * @param dev           Device descriptor
 * @param[out] raw_data Raw data
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_get_rdata(ads130e08_t *dev, ads130e08_raw_data_t *raw_data);

/**
 * @brief Converts raw adc value to voltage
 *
 * @param raw        Raw adc value
 * @param gain       Channel gain
 * @param[out] volts Voltage value
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_convert_raw_to_voltage(int16_t raw, uint8_t gain, float *volts);

/**
 * @brief Set fault detect control
 *
 * @param dev        Pointer to device descriptor
 * @param fault_mode Fault mode
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_set_fault_detect_control(ads130e08_t *dev, ads130e08_fault_threshold_t fault_mode);

/**
 * @brief Get fault detect control
 *
 * @param dev             Pointer to device descriptor
 * @param[out] fault_mode Fault mode
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_get_fault_detect_control(ads130e08_t *dev, ads130e08_fault_threshold_t *fault_mode);

/**
 * @brief Set channel configuration
 *
 * @param dev     Pointer to device descriptor
 * @param channel Channel
 * @param config  Channel configuration
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_set_channel_config(ads130e08_t *dev, ads130e08_channel_t channel,
    ads130e08_channel_config_t config);

/**
 * @brief Get channel configuration
 *
 * @param dev         Pointer to device descriptor
 * @param channel     Channel
 * @param[out] config Channel configuration
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_get_channel_config(ads130e08_t *dev, ads130e08_channel_t channel,
    ads130e08_channel_config_t *config);

/**
 * @brief Set GPIO pin mode
 *
 * @param dev       Pointer to device descriptor
 * @param gpio_pin  GPIO pin number
 * @param gpio_mode GPIO pin mode
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_set_gpio_pin_mode(ads130e08_t *dev, ads130e08_gpio_pin_t gpio_pin, ads130e08_gpio_mode_t gpio_mode);

/**
 * @brief Get GPIO pin mode
 *
 * @param dev            Pointer to device descriptor
 * @param gpio_pin       GPIO pin number
 * @param[out] gpio_mode GPIO pin mode
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_get_gpio_pin_mode(ads130e08_t *dev, ads130e08_gpio_pin_t gpio_pin,
    ads130e08_gpio_mode_t *gpio_mode);

/**
 * @brief Set GPIO pin level
 *
 * @param dev        Pointer to device descriptor
 * @param gpio_pin   GPIO pin number
 * @param gpio_level GPIO pin level
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_set_gpio_pin_level(ads130e08_t *dev, ads130e08_gpio_pin_t gpio_pin,
    ads130e08_gpio_level_t gpio_level);

/**
 * @brief Get GPIO pin level
 *
 * @param dev             Pointer to device descriptor
 * @param gpio_pin        GPIO pin number
 * @param[out] gpio_level GPIO pin level
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_get_gpio_pin_level(ads130e08_t *dev, ads130e08_gpio_pin_t gpio_pin,
    ads130e08_gpio_level_t *gpio_level);

/**
 * @brief Run automatical fault detection cycle
 *
 * @param dev Device descriptor
 * @param [out] fault_statp See datasheet
 * @param [out] fault_statn See datasheet
 * @return `ESP_OK` on success
 */
esp_err_t ads130e08_detect_fault_auto(ads130e08_t *dev, uint8_t *fault_statp, uint8_t *fault_statn);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ADS130E08_H__ */

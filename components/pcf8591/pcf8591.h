/**
 * @file pcf8591.h
 * @defgroup pcf8591 pcf8591
 * @{
 *
 * ESP-IDF driver for 8-bit analog-to-digital conversion and
 * an 8-bit digital-to-analog conversion PCF8591
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2017 Pham Ngoc Thanh <pnt239@gmail.com>\n
 * Copyright (C) 2017, 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __PCF8591_H__
#define __PCF8591_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCF8591_DEFAULT_ADDRESS 0x48

/**
 * Analog inputs configuration, see datasheet
 */
typedef enum {
    PCF8591_IC_4_SINGLES = 0,   //!< Four single-ended inputs
    PCF8591_IC_DIFF,            //!< Three differential inputs
    PCF8591_IC_2_SINGLES_DIFF,  //!< Two single-ended and differnetial mixed
    PCF8591_IC_2_DIFFS          //!< Two differential inputs
} pcf8591_input_conf_t;

/**
 * Initialize device descriptior
 * @param dev Device descriptor
 * @param addr I2C device address
 * @param port I2C port number
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t pcf8591_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * Free device descriptor
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf8591_free_desc(i2c_dev_t *dev);

/**
 * Read input value of an analog pin.
 * @param[in] dev Device descriptor
 * @param[in] conf Analog inputs configuration
 * @param[in] channel Analog channel
 * @param[out] value Analog value
 * @return `ESP_OK` on success
 */
esp_err_t pcf8591_read(i2c_dev_t *dev, pcf8591_input_conf_t conf, uint8_t channel, uint8_t *value);

/**
 * Write value to analog output
 * @param[in] dev Device descriptor
 * @param[in] value DAC value
 * @return `ESP_OK` on success
 */
esp_err_t pcf8591_write(i2c_dev_t *dev, uint8_t value);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PCF8591_H__ */

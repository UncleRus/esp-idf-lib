
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
 * ----------------------------------------------------------------------------
 * Portions copyright (C) 2000 Dallas Semiconductor Corporation, under the
 * following additional terms:
 *
 * Except as contained in this notice, the name of Dallas Semiconductor
 * shall not be used except as stated in the Dallas Semiconductor
 * Branding Policy.
 */

/**
 * @file ds2438_conf.c
 * @author Raghav Jha (raghavjha1531@gmail.com)
 * @defgroup ds2438 ds2438
 * @{
 * @brief ESP-IDF driver for ds2438_conf IC.
 *
 * Ported from arduino library <https://github.com/jbechter/arduino-onewire-DS2438>
 *
 * @version 1.0.1
 * @copyright Copyright (c) 2022 Raghav Jha (raghavjha1531@gmail.com)
 */

#ifndef _DS2438_H_
#define _DS2438_H_

#include <driver/gpio.h>
#include <onewire.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        DS2438_MODE_CHA = 0x01,         /*!< Channel A for ADC Input VAD */
        DS2438_MODE_CHB = 0x02,         /*!< Channel B for ADC Input VDD */
        DS2438_MODE_TEMPERATURE = 0x04, /*!< Channel A for ADC Input VAD */
    } mode_type_t;

    /**
     * @brief Configuration of the ds2438.
     *
     */
    typedef struct
    {
        mode_type_t mode;     /*!< ds2438 operation mode */
        uint32_t onewire_pin; /*!< ds248 one wire pin */
        onewire_addr_t addr;  /*!< ds2438 address */
    } ds2438_dev_t;

    /**
     * @brief structure to store the ds2438 data.
     *
     */
    typedef struct
    {
        double temperatue;
        float voltage;
    } ds2438_data_t;

    /**
     * @brief get temperature
     *
     * @param[in] ds2438_conf   ds2438 configuration
     * @param[out] temp        temperature
     * @return esp_err_t
     */
    esp_err_t ds2438_get_temperature(const ds2438_dev_t *ds2438_conf, double *temp);

    /**
     * @brief get voltage
     *
     * @param[in] ds2438_conf   ds2438 configuration
     * @param[out] voltage      voltage
     * @return
     *
     */
    esp_err_t ds2438_get_voltage(const ds2438_dev_t *ds2438_conf, float *voltage);

    /**
     * @brief To get the device addr (64 bit unsigned int).
     *
     * @param ds2438_conf   ds2438 configuration
     * @return
     *      - one wire address
     *      - else ONEWIRE_NONE
     *
     */
    onewire_addr_t ds2438_get_device_addr(const ds2438_dev_t *ds2438_conf);

    /**
     * @brief Update the temperate and voltage values by reading from modules.
     *
     * @param ds2438_conf[in]   ds2438 configuration
     * @param data[out]         read data.
     * @return
     *      - ESP_OK: If temperature and voltage read successfully.
     *      - ESP_FAIL: if mode is wrong or failed to read temp/voltage.
     */
    esp_err_t ds2438_update(const ds2438_dev_t *ds2438_conf, ds2438_data_t *data);

    /**
     * @brief Get unique ID of the device ( buffer )
     *
     * @param id    : buffer to save the id.
     * @return
     *      - ESP_OK
     */
    esp_err_t ds2438_get_uniqueID(uint8_t *id);

    /**
     * @brief Initialize ds2438 sensor by reading addr.
     *
     * @param ds2438_conf[in]   ds2438 configuration
     * @return
     *      - ESP_OK:   device found.
     *      - ESP_FAIL: device not found.
     */
    esp_err_t ds2438_init(ds2438_dev_t *ds2438_conf);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* _DS2438_H_ */
/*
 * Copyright (c) 2020 Erriez <https://github.com/Erriez>
 * Copyright (c) 2021 David Douard <david.douard@sdfa3.org>
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
 * @file mhz19b.h
 * @defgroup mhz19b mhz19b
 * @{
 *
 * ESP-IDF driver for MH-Z19B NDIR CO2 sensor connected to UART
 *
 * Inspired from https://github.com/Erriez/ErriezMHZ19B
 *
 * Copyright (c) 2020 Erriez <https://github.com/Erriez>\n
 * Copyright (c) 2021 David Douard <david.douard@sdfa3.org>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MHZ19B_H__
#define __MHZ19B_H__

#include <stdbool.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MHZ19B sensor device data structure
 */
typedef struct
{
    uart_port_t uart_port;  ///< UART port used to communicate
    uint8_t *buf;           ///< read buffer attached to this device
    int16_t last_value;     ///< last read value
    int64_t last_ts;        ///< timestamp of the last sensor co2 level reading
} mhz19b_dev_t;

//! 3 minutes warming-up time after power-on before valid data returned
#define MHZ19B_WARMING_UP_TIME_MS       (3UL * 60000UL)
#define MHZ19B_WARMING_UP_TIME_US       (3UL * 60000UL * 1000UL)

//! Minimum response time between CO2 reads (EXPERIMENTALLY DEFINED)
#define MHZ19B_READ_INTERVAL_MS         (5UL * 1000UL)

//! Fixed 9 Bytes response
#define MHZ19B_SERIAL_RX_BYTES          9
//! 128 is the minimal value the UART driver will accept (at least on esp32)
#define MHZ19B_SERIAL_BUF_LEN           128

//! Response timeout between 15..120 ms at 9600 baud works reliable for all commands
#define MHZ19B_SERIAL_RX_TIMEOUT_MS     120

// Documented commands
#define MHZ19B_CMD_SET_AUTO_CAL         0x79 ///< set auto calibration on/off
#define MHZ19B_CMD_READ_CO2             0x86 ///< read CO2 concentration
#define MHZ19B_CMD_CAL_ZERO_POINT       0x87 ///< calibrate zero point at 400ppm
#define MHZ19B_CMD_CAL_SPAN_PIONT       0x88 ///< calibrate span point (NOT IMPLEMENTED)
#define MHZ19B_CMD_SET_RANGE            0x99 ///< set detection range

// Not documented commands
#define MHZ19B_CMD_GET_AUTO_CAL         0x7D ///< get auto calibration status (NOT DOCUMENTED)
#define MHZ19B_CMD_GET_RANGE            0x9B ///< get range detection (NOT DOCUMENTED)
#define MHZ19B_CMD_GET_VERSION          0xA0 ///< get firmware version (NOT DOCUMENTED)

/**
 * @brief PPM range
 */
typedef enum {
    MHZ19B_RANGE_2000 = 2000,            ///< 2000 ppm
    MHZ19B_RANGE_5000 = 5000,            ///< 5000 ppm (Default)
} mhz19b_range_t;


/**
 * @brief Initialize device descriptor
 *
 * @param dev Pointer to the sensor device data structure
 * @param uart_port UART poert number
 * @param tx_gpio GPIO pin number for TX
 * @param rx_gpio GPIO pin number for RX
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_init(mhz19b_dev_t *dev, uart_port_t uart_port, gpio_num_t tx_gpio, gpio_num_t rx_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Pointer to the sensor device data structure
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_free(mhz19b_dev_t *dev);

/**
 * @brief Detect sensor by checking range response
 *
 * @param dev Pointer to the sensor device data structure
 *
 * @return true if sensor is detected
 */
bool mhz19b_detect(mhz19b_dev_t *dev);

/**
 * @brief Check if sensor is warming up
 *
 * @param dev Pointer to the sensor device data structure
 * @param smart_warming_up Smart check
 *
 * @return true if sensor is warming up
 */
bool mhz19b_is_warming_up(mhz19b_dev_t *dev, bool smart_warming_up);

/**
 * @brief Check minimum interval between CO2 reads
 *
 * Not described in the datasheet, but it is the same frequency as the built-in LED blink.
 *
 * @param dev Pointer to the sensor device data structure
 *
 * @return true if ready to call ::mhz19b_read_co2()
 */
bool mhz19b_is_ready(mhz19b_dev_t *dev);

/**
 * @brief Read CO2 from sensor
 *
 * @param dev Pointer to the sensor device data structure
 * @param[out] co2 CO2 level
 *      - < 0: MH-Z19B response error codes.
 *      - 0..399 ppm: Incorrect values. Minimum value starts at 400ppm outdoor fresh air.
 *      - 400..1000 ppm: Concentrations typical of occupied indoor spaces with good air exchange.
 *      - 1000..2000 ppm: Complaints of drowsiness and poor air quality. Ventilation is required.
 *      - 2000..5000 ppm: Headaches, sleepiness and stagnant, stale, stuffy air. Poor concentration, loss of
 *        attention, increased heart rate and slight nausea may also be present.
 *      - Higher values are extremely dangerous and cannot be measured.
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_read_co2(mhz19b_dev_t *dev, int16_t *co2);

/**
 * @brief Get firmware version (NOT DOCUMENTED)
 *
 * @details
 *      This is an undocumented command, but most sensors returns ASCII "0430 or "0443".
 *
 * @param dev Pointer to the sensor device data structure
 * @param[out] version
 *      Returned character pointer to version (must be at least 5 Bytes)\n
 *      Only valid when return is set to ESP_OK.
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_get_version(mhz19b_dev_t *dev, char *version);

/**
 * @brief Set CO2 range
 *
 * @param dev Pointer to the sensor device data structure
 * @param range Range of the sensor (2000 or 5000, in ppm)
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_set_range(mhz19b_dev_t *dev, mhz19b_range_t range);

/**
 * @brief Get CO2 range in PPM (NOT DOCUMENTED)
 *
 * @details
 *      This function verifies valid read ranges of 2000 or 5000 ppm.\n
 *      Note: Other ranges may be returned, but are undocumented and marked as invalid.
 *
 * @param dev Pointer to the sensor device data structure
 * @param range Current value of the range of the sensor (output)
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_get_range(mhz19b_dev_t *dev, uint16_t *range);

/**
 * @brief Enable or disable automatic calibration
 *
 * @param dev Pointer to the sensor device data structure
 * @param calibration_on
 *      - true: Automatic calibration on.
 *      - false: Automatic calibration off.
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_set_auto_calibration(mhz19b_dev_t *dev, bool calibration_on);

/**
 * @brief Get status of automatic calibration (NOT DOCUMENTED)
 *
 * @param dev Pointer to the sensor device data structure
 * @param[out] calibration_on Automatic calibration status
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_get_auto_calibration(
	mhz19b_dev_t *dev, bool *calibration_on); // (NOT DOCUMENTED)

/**
 * @brief Start Zero Point Calibration manually at 400ppm
 *
 * @details
 *      The sensor must be powered-up for at least 20 minutes in fresh air at 400ppm room
 *      temperature. Then call this function once to execute self calibration.\n
 *      Recommended to use this function when auto calibration is off.
 *
 * @param dev Pointer to the sensor device data structure
 *
 * @return ESP_OK on success
 */
esp_err_t mhz19b_start_calibration(mhz19b_dev_t *dev);

/**
 * @brief Send serial command to sensor and read response
 *
 * @details
 *      Send command to sensor and read response, protected with a receive timeout.\n
 *      Result is available in the device descriptor buffer.
 *
 * @param dev Pointer to the sensor device data structure
 * @param cmd Command Byte
 * @param b3 Byte 3 (default 0)
 * @param b4 Byte 4 (default 0)
 * @param b5 Byte 5 (default 0)
 * @param b6 Byte 6 (default 0)
 * @param b7 Byte 7 (default 0)
 */
esp_err_t mhz19b_send_command(mhz19b_dev_t *dev, uint8_t cmd,
							  uint8_t b3, uint8_t b4, uint8_t b5,
							  uint8_t b6, uint8_t b7);

/**
 * @brief Calculate CRC on 8 data Bytes buffer
 *
 * @param data Buffer pointer to calculate CRC.
 * @return Calculated 8-bit CRC.
 */
uint8_t mhz19b_calc_crc(uint8_t *data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

/**@}*/

#endif /* __MHZ19B_H__ */

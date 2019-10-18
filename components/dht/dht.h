/**
 * @file dht.h
 * @defgroup dht dht
 * @{
 *
 * ESP-IDF driver for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), Itead Si7021
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016 Jonathan Hartsuiker <https://github.com/jsuiker>\n
 * Copyright (C) 2018 Ruslan V. Uss <https://github.com/UncleRus>\n
 *
 * BSD Licensed as described in the file LICENSE
 *
 * @note A suitable pull-up resistor should be connected to the selected GPIO line
 *
 */
#ifndef __DHT_H__
#define __DHT_H__

#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Sensor type
 */
typedef enum
{
    DHT_TYPE_DHT11 = 0,   //!< DHT11
    DHT_TYPE_AM2301,      //!< AM2301 (DHT21, DHT22, AM2302, AM2321)
    DHT_TYPE_SI7021       //!< Itead Si7021
} dht_sensor_type_t;

/**
 * @brief Read data from sensor on specified pin.
 * Humidity and temperature are returned as integers.
 * For example: humidity=625 is 62.5 %
 *              temperature=24.4 is 24.4 degrees Celsius
 *
 * @param[in] sensor_type DHT11 or DHT22
 * @param[in] pin GPIO pin connected to sensor OUT
 * @param[out] humidity Humidity, percents * 10
 * @param[out] temperature Temperature, degrees Celsius * 10
 * @return `ESP_OK` on success
 */
esp_err_t dht_read_data(dht_sensor_type_t sensor_type, gpio_num_t pin,
        int16_t *humidity, int16_t *temperature);

/**
 * @brief Read data from sensor on specified pin.
 * Humidity and temperature are returned as floats.
 *
 * @param[in] sensor_type DHT11 or DHT22
 * @param[in] pin GPIO pin connected to sensor OUT
 * @param[out] humidity Humidity, percents
 * @param[out] temperature Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t dht_read_float_data(dht_sensor_type_t sensor_type, gpio_num_t pin,
        float *humidity, float *temperature);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __DHT_H__

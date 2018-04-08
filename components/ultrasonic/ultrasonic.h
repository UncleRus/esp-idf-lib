/**
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and so on
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#ifndef EXTRAS_ULTRASONIC_H_
#define EXTRAS_ULTRASONIC_H_

#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ERR_ULTRASONIC_PING         0x200
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202

/**
 * Device descriptor
 */
typedef struct
{
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
} ultrasonic_sensor_t;

/**
 * Init ranging module
 * \param dev Pointer to the device descriptor
 */
void ultrasonic_init(const ultrasonic_sensor_t *dev);

/**
 * Measure distance
 * \param dev Pointer to the device descriptor
 * \param max_distance Maximal distance to measure, centimeters
 * \return Distance in centimeters or ULTRASONIC_ERROR_xxx if error occured
 */
esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t *dev, uint32_t max_distance, uint32_t *distance);

#ifdef __cplusplus
}
#endif

#endif /* EXTRAS_ULTRASONIC_H_ */

/**
 * @file ultrasonic.c
 *
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and so on
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "ultrasonic.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>

#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 6000
#define ROUNDTRIP 58

#if defined(CONFIG_IDF_TARGET_ESP32)
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

#elif defined(CONFIG_IDF_TARGET_ESP8266)
#define PORT_ENTER_CRITICAL portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL()
#endif

static inline uint32_t get_time_us()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec;
}

#define timeout_expired(start, len) ((uint32_t)(get_time_us() - (start)) >= (len))

#define RETURN_CRTCAL(RES) do { PORT_EXIT_CRITICAL; return RES; } while(0)

void ultrasonic_init(const ultrasonic_sensor_t *dev)
{
    gpio_set_direction(dev->trigger_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->echo_pin, GPIO_MODE_INPUT);

    gpio_set_level(dev->trigger_pin, 0);
}

esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t *dev, uint32_t max_distance, uint32_t *distance)
{
    if (!distance)
        return ESP_ERR_INVALID_ARG;

    PORT_ENTER_CRITICAL;

    // Ping: Low for 2..4 us, then high 10 us
    gpio_set_level(dev->trigger_pin, 0);
    ets_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(dev->trigger_pin, 1);
    ets_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(dev->trigger_pin, 0);

    // Previous ping isn't ended
    if (gpio_get_level(dev->echo_pin))
        RETURN_CRTCAL(ESP_ERR_ULTRASONIC_PING);

    // Wait for echo
    uint32_t start = get_time_us();
    while (!gpio_get_level(dev->echo_pin))
    {
        if (timeout_expired(start, PING_TIMEOUT))
            RETURN_CRTCAL(ESP_ERR_ULTRASONIC_PING_TIMEOUT);
    }

    // got echo, measuring
    uint32_t echo_start = get_time_us();
    uint32_t time = echo_start;
    uint32_t meas_timeout = echo_start + max_distance * ROUNDTRIP;
    while (gpio_get_level(dev->echo_pin))
    {
        time = get_time_us();
        if (timeout_expired(echo_start, meas_timeout))
            RETURN_CRTCAL(ESP_ERR_ULTRASONIC_ECHO_TIMEOUT);
    }
    PORT_EXIT_CRITICAL;

    *distance = (time - echo_start) / ROUNDTRIP;

    return ESP_OK;
}

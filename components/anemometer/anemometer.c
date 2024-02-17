/*
 * Copyright (c) 2024 Jakub Turek <qb4.dev@gmail.com>
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
 * @file anemometer.c
 *
 * ESP-IDF driver for impulse wind speed sensors(anemometers)
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "anemometer.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <esp_idf_lib_helpers.h>

#ifdef CONFIG_IDF_TARGET_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

#elif CONFIG_IDF_TARGET_ESP8266
#define PORT_ENTER_CRITICAL portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL()

#else
#error cannot identify the target
#endif

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define RETURN_CRITICAL(RES) do { PORT_EXIT_CRITICAL; return RES; } while(0)

typedef struct {
	gpio_num_t input_pin;      //!< GPIO input pin
	float sf;                  //!< scale factor
	uint32_t pps;              //!< measured pulses count per second
	TickType_t init_tick;      //!< measurement init tick
} anemometer_priv_t;

static inline void pps_iterate(anemometer_priv_t *priv, TickType_t ticks)
{
    if(ticks - priv->init_tick > pdMS_TO_TICKS(1000)){
        priv->init_tick = ticks;
        priv->pps = 0;
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    anemometer_priv_t *priv = (anemometer_priv_t *)arg;
    TickType_t ticks = xTaskGetTickCountFromISR();
    pps_iterate(arg,ticks);
    priv->pps++;
}

anemometer_t anemometer_init(const anemometer_config_t *conf)
{
	gpio_config_t io_conf;
    anemometer_priv_t *priv;

    priv = (anemometer_priv_t *)calloc(1,sizeof(anemometer_priv_t));
    if(priv == NULL){
        return NULL;
    }

    priv->input_pin = conf->input_pin;
    priv->sf = conf->scale_factor ? conf->scale_factor : ANEMOMETER_DEFAULT_SF;

    /* setup GPIO */
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1 << priv->input_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    /* enable interrupts */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(priv->input_pin, gpio_isr_handler, (void *) priv);
    return priv;
}

esp_err_t anemometer_deinit(anemometer_t *anemometer)
{
    CHECK_ARG(anemometer);
    anemometer_priv_t *priv = (anemometer_priv_t *)anemometer;

    gpio_isr_handler_remove(priv->input_pin);
    free(priv);
    return ESP_OK;
}

esp_err_t anemometer_get_wind_speed(anemometer_t *anemometer, float *speed)
{
    CHECK_ARG(anemometer);
    CHECK_ARG(speed);
    anemometer_priv_t *priv = (anemometer_priv_t *)anemometer;
    TickType_t ticks = xTaskGetTickCount();

    PORT_ENTER_CRITICAL;
    pps_iterate(priv,ticks);
    *speed = priv->sf * priv->pps;
    PORT_EXIT_CRITICAL;
    return ESP_OK;
}





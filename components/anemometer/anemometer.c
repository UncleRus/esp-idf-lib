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
 * Copyright (c) 2024 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "anemometer.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>

#if HELPER_TARGET_IS_ESP32
#include <driver/pulse_cnt.h>
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL(&mux)

#elif HELPER_TARGET_IS_ESP8266
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL()

#else
#error cannot identify the target
#endif

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

typedef struct {
    gpio_num_t input_pin;          //!< GPIO input pin
    float sf;                      //!< scale factor
    int pps;                       //!< measured pulses count per second
    esp_timer_handle_t timer;      //!< periodic timer to reset pps count
#if HELPER_TARGET_IS_ESP8266
    int pulse_count;               //!< pulses counter
#elif HELPER_TARGET_IS_ESP32
    pcnt_unit_handle_t pcnt_unit;  //!< hardware pulse counter
    pcnt_channel_handle_t pcnt_ch; //!< hardware pulse counter channel
#endif
} anemometer_priv_t;

#if HELPER_TARGET_IS_ESP8266
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    anemometer_priv_t *priv = (anemometer_priv_t *)arg;
    priv->pulse_count++;
}

static esp_err_t anemometer_pulse_counter_init(anemometer_priv_t *priv)
{
    /* enable interrupts */
    esp_err_t rc = gpio_install_isr_service(0);
    if (rc != ESP_OK && rc != ESP_ERR_INVALID_STATE){
        return rc;
    }
    /* setup GPIO */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1 << priv->input_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(priv->input_pin, gpio_isr_handler,priv));
    return ESP_OK;
}

static esp_err_t anemometer_pulse_counter_deinit(anemometer_priv_t *priv)
{
    ESP_ERROR_CHECK(gpio_isr_handler_remove(priv->input_pin));
    return ESP_OK;
}

static void timer_event_callback(void *arg)
{
    anemometer_priv_t *priv = (anemometer_priv_t *)arg;

    PORT_ENTER_CRITICAL();
    priv->pps = priv->pulse_count;
    priv->pulse_count = 0;
    PORT_EXIT_CRITICAL();
}

#elif HELPER_TARGET_IS_ESP32
static esp_err_t anemometer_pulse_counter_init(anemometer_priv_t *priv)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = 2048,
        .low_limit = -1
    };

    pcnt_chan_config_t ch_config = {
        .edge_gpio_num = priv->input_pin,
        .level_gpio_num = -1
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config,&priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_channel(priv->pcnt_unit,&ch_config,&priv->pcnt_ch));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(priv->pcnt_ch,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_unit_enable(priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(priv->pcnt_unit));
    return ESP_OK;
}

static esp_err_t anemometer_pulse_counter_deinit(anemometer_priv_t *priv)
{
    ESP_ERROR_CHECK(pcnt_unit_stop(priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_disable(priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_del_channel(priv->pcnt_ch));
    ESP_ERROR_CHECK(pcnt_del_unit(priv->pcnt_unit));
    return ESP_OK;
}

static void timer_event_callback(void *arg)
{
    anemometer_priv_t *priv = (anemometer_priv_t *)arg;

    pcnt_unit_get_count(priv->pcnt_unit,&priv->pps);
    pcnt_unit_clear_count(priv->pcnt_unit);
}
#endif

static esp_err_t anemometer_timer_setup(anemometer_priv_t *priv)
{
    esp_timer_create_args_t timer_args = {
        .name = "anemometer",
        .dispatch_method = ESP_TIMER_TASK,
        .callback = timer_event_callback,
        .arg = priv,
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args,&priv->timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(priv->timer,1000*1000));
    return ESP_OK;
}

esp_err_t anemometer_init(const anemometer_config_t *conf, anemometer_t *anemometer)
{
    CHECK_ARG(conf);
    CHECK_ARG(anemometer);

    anemometer_priv_t *priv;
    esp_err_t rc;

    priv = (anemometer_priv_t *)calloc(1,sizeof(anemometer_priv_t));
    if(priv == NULL){
        return ESP_ERR_NO_MEM;
    }

    priv->input_pin = conf->input_pin;
    priv->sf = conf->scale_factor ? conf->scale_factor : ANEMOMETER_DEFAULT_SF;

    rc = anemometer_pulse_counter_init(priv);
    if (rc != ESP_OK){
        free(priv);
        return rc;
    }

    rc = anemometer_timer_setup(priv);
    if (rc != ESP_OK){
        anemometer_pulse_counter_deinit(priv);
        free(priv);
        return rc;
    }
    *anemometer = priv;
    return ESP_OK;
}

esp_err_t anemometer_deinit(anemometer_t *anemometer)
{
    CHECK_ARG(anemometer);
    anemometer_priv_t *priv = (anemometer_priv_t *)anemometer;

    ESP_ERROR_CHECK(esp_timer_stop(priv->timer));
    anemometer_pulse_counter_deinit(priv);
    free(priv);
    return ESP_OK;
}

esp_err_t anemometer_get_wind_speed(anemometer_t *anemometer, float *speed)
{
    CHECK_ARG(anemometer);
    CHECK_ARG(speed);

    anemometer_priv_t *priv = (anemometer_priv_t *)anemometer;

    PORT_ENTER_CRITICAL();
    *speed = priv->sf * priv->pps;
    PORT_EXIT_CRITICAL();
    return ESP_OK;
}


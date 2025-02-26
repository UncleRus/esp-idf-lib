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
 * @file impulse_sensor.c
 *
 * ESP-IDF driver for impulse sensors
 *
 * Copyright (c) 2024 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "impulse_sensor.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_idf_lib_helpers.h>

#if defined(HELPER_TARGET_IS_ESP32) \
    && !defined(CONFIG_IDF_TARGET_ESP32C2) \
    && !defined(CONFIG_IDF_TARGET_ESP32C3) \
    && !defined(CONFIG_IDF_TARGET_ESP32C61)


#define ESP_PCNT_SUPPORTED (1)
#include <driver/pulse_cnt.h>
#else
#define ESP_PCNT_SUPPORTED (0)
#endif

#if HELPER_TARGET_IS_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL()  portEXIT_CRITICAL(&mux)

#elif HELPER_TARGET_IS_ESP8266
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL()  portEXIT_CRITICAL()

#else
#error cannot identify the target
#endif

#define CHECK_ARG(VAL)                                                                                                                                                                                 \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (!(VAL))                                                                                                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                                                                                                \
    }                                                                                                                                                                                                  \
    while (0)

typedef struct
{
    gpio_num_t input_pin;     //!< GPIO input pin
    float sf;                 //!< scale factor
    int pps;                  //!< measured pulses count per second
    esp_timer_handle_t timer; //!< periodic timer to reset pps count

#if ESP_PCNT_SUPPORTED
    pcnt_unit_handle_t pcnt_unit;  //!< hardware pulse counter
    pcnt_channel_handle_t pcnt_ch; //!< hardware pulse counter channel
#else
    int pulse_count; //!< pulses counter
#endif
} imp_sensor_priv_t;

#if ESP_PCNT_SUPPORTED

static esp_err_t pulse_counter_init(imp_sensor_priv_t *priv)
{
    pcnt_unit_config_t unit_config = { .high_limit = SHRT_MAX, .low_limit = -1 };

    pcnt_chan_config_t ch_config = { .edge_gpio_num = priv->input_pin, .level_gpio_num = -1 };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_channel(priv->pcnt_unit, &ch_config, &priv->pcnt_ch));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(priv->pcnt_ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_unit_enable(priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(priv->pcnt_unit));
    return ESP_OK;
}

static esp_err_t pulse_counter_deinit(imp_sensor_priv_t *priv)
{
    ESP_ERROR_CHECK(pcnt_unit_stop(priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_disable(priv->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_del_channel(priv->pcnt_ch));
    ESP_ERROR_CHECK(pcnt_del_unit(priv->pcnt_unit));
    return ESP_OK;
}

static void timer_event_callback(void *arg)
{
    imp_sensor_priv_t *priv = (imp_sensor_priv_t *)arg;

    pcnt_unit_get_count(priv->pcnt_unit, &priv->pps);
    pcnt_unit_clear_count(priv->pcnt_unit);
}

#else

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    imp_sensor_priv_t *priv = (imp_sensor_priv_t *)arg;
    priv->pulse_count++;
}

static esp_err_t pulse_counter_init(imp_sensor_priv_t *priv)
{
    /* enable interrupts */
    esp_err_t rc = gpio_install_isr_service(0);
    if (rc != ESP_OK && rc != ESP_ERR_INVALID_STATE)
    {
        return rc;
    }
    /* setup GPIO */
    gpio_config_t io_conf = { .intr_type = GPIO_INTR_POSEDGE, .pin_bit_mask = (1 << priv->input_pin), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(priv->input_pin, gpio_isr_handler, priv));
    return ESP_OK;
}

static esp_err_t pulse_counter_deinit(imp_sensor_priv_t *priv)
{
    ESP_ERROR_CHECK(gpio_isr_handler_remove(priv->input_pin));
    return ESP_OK;
}

static void timer_event_callback(void *arg)
{
    imp_sensor_priv_t *priv = (imp_sensor_priv_t *)arg;

    PORT_ENTER_CRITICAL();
    priv->pps = priv->pulse_count;
    priv->pulse_count = 0;
    PORT_EXIT_CRITICAL();
}

#endif

static esp_err_t timer_setup(imp_sensor_priv_t *priv, const uint32_t period)
{
    esp_timer_create_args_t timer_args = {
        .name = "imp-sensor",
        .dispatch_method = ESP_TIMER_TASK,
        .callback = timer_event_callback,
        .arg = priv,
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &priv->timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(priv->timer, period * 1000));
    return ESP_OK;
}

esp_err_t imp_sensor_init(const imp_sensor_config_t *conf, imp_sensor_t *imp_sensor)
{
    CHECK_ARG(conf);
    CHECK_ARG(imp_sensor);

    imp_sensor_priv_t *priv;
    esp_err_t rc;
    const uint32_t timer_period_ms = conf->meas_period ? conf->meas_period : IMP_SENSOR_DEFAULT_MEAS_PERIOD;

    priv = (imp_sensor_priv_t *)calloc(1, sizeof(imp_sensor_priv_t));
    if (priv == NULL)
        return ESP_ERR_NO_MEM;

    priv->input_pin = conf->input_pin;
    priv->sf = conf->scale_factor ? conf->scale_factor : IMP_SENSOR_DEFAULT_SF;

    rc = pulse_counter_init(priv);
    if (rc != ESP_OK)
    {
        free(priv);
        return rc;
    }

    rc = timer_setup(priv, timer_period_ms);
    if (rc != ESP_OK)
    {
        pulse_counter_deinit(priv);
        free(priv);
        return rc;
    }
    *imp_sensor = priv;
    return ESP_OK;
}

esp_err_t imp_sensor_deinit(imp_sensor_t *imp_sensor)
{
    CHECK_ARG(imp_sensor);
    imp_sensor_priv_t *priv = (imp_sensor_priv_t *)imp_sensor;

    ESP_ERROR_CHECK(esp_timer_stop(priv->timer));
    pulse_counter_deinit(priv);
    free(priv);
    return ESP_OK;
}

esp_err_t imp_sensor_get_value(imp_sensor_t *imp_sensor, float *value)
{
    CHECK_ARG(imp_sensor);
    CHECK_ARG(value);

    imp_sensor_priv_t *priv = (imp_sensor_priv_t *)imp_sensor;

    PORT_ENTER_CRITICAL();
    *value = priv->sf * priv->pps;
    PORT_EXIT_CRITICAL();
    return ESP_OK;
}

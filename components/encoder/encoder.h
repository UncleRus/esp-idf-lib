/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file encoder.h
 * @defgroup encoder encoder
 * @{
 *
 * ESP-IDF HW timer-based driver for rotary encoders
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <esp_err.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Button state
 */
typedef enum {
    RE_BTN_RELEASED = 0,      //!< Button currently released
    RE_BTN_PRESSED = 1,       //!< Button currently pressed
    RE_BTN_LONG_PRESSED = 2   //!< Button currently long pressed
} rotary_encoder_btn_state_t;

/**
 * Rotary encoder descriptor
 */
typedef struct
{
    gpio_num_t pin_a, pin_b, pin_btn; //!< Encoder pins. pin_btn can be >= GPIO_NUM_MAX if no button used
    uint8_t code;
    uint16_t store;
    size_t index;
    uint64_t btn_pressed_time_us;
    rotary_encoder_btn_state_t btn_state;
} rotary_encoder_t;

/**
 * Event type
 */
typedef enum {
    RE_ET_CHANGED = 0,      //!< Encoder turned
    RE_ET_BTN_RELEASED,     //!< Button released
    RE_ET_BTN_PRESSED,      //!< Button pressed
    RE_ET_BTN_LONG_PRESSED, //!< Button long pressed (press time (us) > RE_BTN_LONG_PRESS_TIME_US)
    RE_ET_BTN_CLICKED       //!< Button was clicked
} rotary_encoder_event_type_t;

/**
 * Event
 */
typedef struct
{
    rotary_encoder_event_type_t type;  //!< Event type
    rotary_encoder_t *sender;          //!< Pointer to descriptor
    int32_t diff;                      //!< Difference between new and old positions (only if type == RE_ET_CHANGED)
} rotary_encoder_event_t;

/**
 * @brief Initialize library
 *
 * @param queue Event queue to send encoder events
 * @return `ESP_OK` on success
 */
esp_err_t rotary_encoder_init(QueueHandle_t queue);

/**
 * @brief Add new rotary encoder
 *
 * @param re Encoder descriptor
 * @return `ESP_OK` on success
 */
esp_err_t rotary_encoder_add(rotary_encoder_t *re);

/**
 * @brief Remove previously added rotary encoder
 *
 * @param re Encoder descriptor
 * @return `ESP_OK` on success
 */
esp_err_t rotary_encoder_remove(rotary_encoder_t *re);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ENCODER_H__ */

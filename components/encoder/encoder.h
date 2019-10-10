/**
 * @file encoder.h
 * @defgroup encoder encoder
 * @{
 *
 * ESP-IDF HW timer-based driver for rotary encoders
 *
 * Copyright (C) 2019 Ruslan V. Uss <unclerus@gmail.com>
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
 * Initialize library
 * @param queue Event queue
 * @return `ESP_OK` on success
 */
esp_err_t rotary_encoder_init(QueueHandle_t queue);

/**
 * Add new rotary encoder
 * @param re Encoder descriptor
 * @return `ESP_OK` on success
 */
esp_err_t rotary_encoder_add(rotary_encoder_t *re);

/**
 * Remove previously added rotary encoder
 * @param re Encoder descriptor
 * @return `ESP_OK` on success
 */
esp_err_t rotary_encoder_remove(rotary_encoder_t *re);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ENCODER_H__ */

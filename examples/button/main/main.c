#include <stdio.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <button.h>

#if HELPER_TARGET_IS_ESP8266
#define BUTTON1_GPIO   14
#define BUTTON2_GPIO   12

#elif HELPER_TARGET_IS_ESP32
#define BUTTON1_GPIO   16
#define BUTTON2_GPIO   17

#else
#error Unknown platform
#endif

static const char *TAG = "button_example";

static const char *states[] = {
    [BUTTON_PRESSED]      = "pressed",
    [BUTTON_RELEASED]     = "released",
    [BUTTON_CLICKED]      = "clicked",
    [BUTTON_PRESSED_LONG] = "pressed long",
};

static button_t btn1, btn2;

static void on_button(button_t *btn, button_state_t state)
{
    ESP_LOGI(TAG, "%s button %s", btn == &btn1 ? "First" : "Second", states[state]);
}

void app_main()
{
    // First button connected between GPIO and GND
    // pressed logic level 0, no autorepeat
    btn1.gpio = BUTTON1_GPIO;
    btn1.pressed_level = 0;
    btn1.internal_pull = true;
    btn1.autorepeat = false;
    btn1.callback = on_button;

    // Second button connected between GPIO and +3.3V
    // pressed logic level 1, autorepeat enabled
    btn2.gpio = BUTTON1_GPIO;
    btn2.pressed_level = 1;
    btn2.internal_pull = true;
    btn2.autorepeat = true;
    btn2.callback = on_button;

    ESP_ERROR_CHECK(button_init(&btn1));
    ESP_ERROR_CHECK(button_init(&btn2));
}


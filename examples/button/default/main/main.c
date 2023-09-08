#include <stdio.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <button.h>

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
    btn1.gpio = CONFIG_EXAMPLE_BUTTON1_GPIO;
    btn1.pressed_level = 0;
    btn1.internal_pull = true;
    btn1.autorepeat = false;
    btn1.callback = on_button;

    // Second button connected between GPIO and +3.3V
    // pressed logic level 1, autorepeat enabled
    btn2.gpio = CONFIG_EXAMPLE_BUTTON2_GPIO;
    btn2.pressed_level = 1;
    btn2.internal_pull = true;
    btn2.autorepeat = true;
    btn2.callback = on_button;

    ESP_ERROR_CHECK(button_init(&btn1));
    ESP_ERROR_CHECK(button_init(&btn2));
}


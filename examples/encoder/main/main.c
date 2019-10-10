#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <encoder.h>
#include <esp_idf_lib_helpers.h>

// Connect common encoder pin to ground
#if HELPER_TARGET_IS_ESP8266
#define RE_A_GPIO   14
#define RE_B_GPIO   12
#define RE_BTN_GPIO 13

#elif HELPER_TARGET_IS_ESP32
#define RE_A_GPIO   16
#define RE_B_GPIO   17
#define RE_BTN_GPIO 5

#else
#error Unknown platform
#endif

// input loop
static esp_event_loop_handle_t input_loop;

// Encoder handler must be static
static rotary_encoder_t re;

// something to change
static int32_t vol = 0;

static void input_handler(void *event_handler_arg, esp_event_base_t event_base,
        int32_t event_id, void *event_data)
{
    rotary_encoder_t *e = *((rotary_encoder_t **)(event_data));

    printf("Got encoder event, diff = %d, btn = %d\n", e->diff, e->btn_state);

    switch (event_id)
    {
        case RE_EVENT_BTN_PRESSED:
            printf("Button pressed\n");
            break;
        case RE_EVENT_BTN_RELEASED:
            printf("Button released\n");
            break;
        case RE_EVENT_BTN_CLICKED:
            printf("Button clicked\n");
            break;
        case RE_EVENT_BTN_LONG_PRESSED:
            printf("Looooong pressed button\n");
            break;
        case RE_EVENT_CHANGED:
            vol += e->diff;
            printf("Volume: %d\n", vol);
            break;
        default:
            break;
    }
}

void app_main()
{
    // Create custom event loop for input events
    esp_event_loop_args_t loop_args = {
         .queue_size = 10,
         .task_name = "INPUT",
         .task_priority = 5,
         .task_stack_size = configMINIMAL_STACK_SIZE * 2,
         .task_core_id = APP_CPU_NUM
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &input_loop));

    // Register handler
    ESP_ERROR_CHECK(esp_event_handler_register_with(input_loop, RE_EVENT, ESP_EVENT_ANY_ID, input_handler, NULL));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(input_loop));

    // Add one encoder
    memset(&re, 0, sizeof(rotary_encoder_t));
    re.pin_a = RE_A_GPIO;
    re.pin_b = RE_B_GPIO;
    re.pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(&re));
}

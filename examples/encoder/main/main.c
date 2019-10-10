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

#define EV_QUEUE_LEN 5

static QueueHandle_t event_queue;
static rotary_encoder_t re;

void test(void *arg)
{
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));

    // Add one
    memset(&re, 0, sizeof(rotary_encoder_t));
    re.pin_a = RE_A_GPIO;
    re.pin_b = RE_B_GPIO;
    re.pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(&re));

    rotary_encoder_event_t e;
    int32_t vol = 0;

    printf("Initial volume level: %d\n", vol);

    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        printf("Got encoder event. type = %d, sender = 0x%08x, diff = %d\n", e.type, (uint32_t)e.sender, e.diff);

        switch (e.type)
        {
            case RE_ET_BTN_PRESSED:
                printf("Button pressed\n");
                break;
            case RE_ET_BTN_RELEASED:
                printf("Button released\n");
                break;
            case RE_ET_BTN_CLICKED:
                printf("Button clicked\n");
                break;
            case RE_ET_BTN_LONG_PRESSED:
                printf("Looooong pressed button\n");
                break;
            case RE_ET_CHANGED:
                vol += e.diff;
                printf("Volume was changed to %d\n", vol);
                break;
            default:
                break;
        }
    }
}

void app_main()
{
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

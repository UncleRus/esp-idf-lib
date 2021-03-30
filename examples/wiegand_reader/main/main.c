#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <wiegand.h>
#include <esp_log.h>

static const char *TAG = "wiegand_reader";

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define D0_GPIO 4
#define D1_GPIO 5
#else
#define D0_GPIO 16
#define D1_GPIO 17
#endif

#define WIEGAND_CARD_FORMAT WIEGAND_H10301 // 26 bit HID (H10301)

static wiegand_reader_t reader;

static void reader_callback(wiegand_reader_t *r)
{
    ESP_LOGI(TAG, "Got card, %d bits, ID: %08x", r->bits, *((uint32_t*)r->buf));
}

void task(void *arg)
{
    ESP_ERROR_CHECK(wiegand_reader_init(&reader, D0_GPIO, D1_GPIO, true, 16, reader_callback));
    while (1)
    {
        ESP_LOGI(TAG, "Waiting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    xTaskCreate(task, "main_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}


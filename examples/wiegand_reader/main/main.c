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

#define READER_BUF_SIZE 4 // 32 bits should be enough for demo

static wiegand_reader_t reader;

static void reader_callback(wiegand_reader_t *r)
{
    // Can use xQueueSend() here
    ESP_LOGI(TAG, "Got card, %d bits, ID: %08x", r->bits, *((uint32_t*)r->buf));
}

void task(void *arg)
{
    // Initialize reader
    ESP_ERROR_CHECK(wiegand_reader_init(&reader, D0_GPIO, D1_GPIO, true, READER_BUF_SIZE, reader_callback));

    while (1)
    {
        ESP_LOGI(TAG, "Doing something...");
        // Can use xQueueReceive() here
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    xTaskCreate(task, "main_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}


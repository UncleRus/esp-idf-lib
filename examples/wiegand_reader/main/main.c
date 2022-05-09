#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <wiegand.h>
#include <esp_log.h>

static const char *TAG = "wiegand_reader";

static wiegand_reader_t reader;

static void reader_callback(wiegand_reader_t *r)
{
    // Can use xQueueSend() here
    ESP_LOGI(TAG, "Got card, %d bits, ID: %08x", r->bits, *((uint32_t*)r->buf));
}

void task(void *arg)
{
    // Initialize reader
    ESP_ERROR_CHECK(wiegand_reader_init(&reader, CONFIG_EXAMPLE_D0_GPIO, CONFIG_EXAMPLE_D0_GPIO,
            true, CONFIG_EXAMPLE_BUF_SIZE, reader_callback, WIEGAND_MSB_FIRST, WIEGAND_LSB_FIRST));

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


#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <wiegand.h>
#include <esp_log.h>
#include <string.h>

static const char *TAG = "wiegand_reader";

static wiegand_reader_t reader;
static QueueHandle_t queue = NULL;

// Single data packet
typedef struct
{
    uint8_t data[CONFIG_EXAMPLE_BUF_SIZE];
    size_t bits;
} data_packet_t;

// callback on new data in reader
static void reader_callback(wiegand_reader_t *r)
{
    // you can decode raw data from reader buffer here, but remember:
    // reader will ignore any new incoming data while executing callback

    // create simple undecoded data packet
    data_packet_t p;
    p.bits = r->bits;
    memcpy(p.data, r->buf, CONFIG_EXAMPLE_BUF_SIZE);

    // Send it to the queue
    xQueueSendToBack(queue, &p, 0);
}

static void task(void *arg)
{
    // Create queue
    queue = xQueueCreate(5, sizeof(data_packet_t));
    if (!queue)
    {
        ESP_LOGE(TAG, "Error creating queue");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    // Initialize reader
    ESP_ERROR_CHECK(wiegand_reader_init(&reader, CONFIG_EXAMPLE_D0_GPIO, CONFIG_EXAMPLE_D1_GPIO,
            true, CONFIG_EXAMPLE_BUF_SIZE, reader_callback, WIEGAND_MSB_FIRST, WIEGAND_LSB_FIRST));

    data_packet_t p;
    while (1)
    {
        ESP_LOGI(TAG, "Waiting for Wiegand data...");
        xQueueReceive(queue, &p, portMAX_DELAY);

        // dump received data
        printf("==========================================\n");
        printf("Bits received: %d\n", p.bits);
        printf("Received data:");
        int bytes = p.bits / 8;
        int tail = p.bits % 8;
        for (size_t i = 0; i < bytes + (tail ? 1 : 0); i++)
            printf(" 0x%02x", p.data[i]);
        printf("\n==========================================\n");
    }
}

void app_main()
{
    xTaskCreate(task, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
}


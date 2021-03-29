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
    wiegand_card_t card;

    // decode card
    if (wiegand_reader_decode(r, WIEGAND_CARD_FORMAT, &card) != ESP_OK)
    {
        ESP_LOGE(TAG, "Unknown format, bits: %d", r->bits);
        return;
    }

    // print card info
    ESP_LOGI(TAG, "[Got card] Issue Level: 0x%02x; Facility Code: 0x%04x; Card Number: 0x%08x; Card Holder: 0x%08x",
            card.issue_level, card.facility, (uint32_t)card.number, card.cardholder);
}

void app_main()
{
    ESP_ERROR_CHECK(wiegand_reader_init(&reader, D0_GPIO, D1_GPIO, false, 8, reader_callback));
}


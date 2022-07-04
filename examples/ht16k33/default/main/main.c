/*
 * Copyright (c) 2022 Timofei Korostelev <timofei_public@dranik.dev>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <ht16k33.h>
#include <i2cdev.h>

static char *tag = "main";

void app_main()
{
    ESP_LOGI(tag, "HT16K33 example");

    ESP_ERROR_CHECK(i2cdev_init());

    i2c_dev_t dev;

    uint8_t all_on[HT16K33_RAM_SIZE_BYTES];
    memset(all_on, 0xFF, HT16K33_RAM_SIZE_BYTES);
    uint8_t all_off[HT16K33_RAM_SIZE_BYTES];
    memset(all_off, 0x00, HT16K33_RAM_SIZE_BYTES);

    while (1) {
        memset(&dev, 0, sizeof(i2c_dev_t));
        ESP_LOGI(tag, "Initializing HT16K33 descriptor.");
        ESP_ERROR_CHECK(ht16k33_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL, HT16K33_DEFAULT_ADDR));

        ESP_LOGI(tag, "Initializing HT16K33 device.");
        ESP_ERROR_CHECK(ht16k33_init(&dev));

        ESP_LOGI(tag, "Turning display on.");
        ESP_ERROR_CHECK(ht16k33_display_setup(&dev, 1, HTK16K33_F_0HZ));

        ESP_LOGI(tag, "Snake");
        uint8_t snake_ram[HT16K33_RAM_SIZE_BYTES];
        memset(snake_ram, 0, HT16K33_RAM_SIZE_BYTES);
        for (int bit = 0; bit < HT16K33_RAM_SIZE_BYTES * 8; bit++) {
            int byte = bit / 8;
            snake_ram[byte] = snake_ram[byte] | 1 << bit % 8;
            ESP_ERROR_CHECK(ht16k33_ram_write(&dev, snake_ram));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        for (int i = 0; i < 2; i++) {
            ESP_LOGI(tag, "OFF");
            ESP_ERROR_CHECK(ht16k33_ram_write(&dev, all_off));
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(tag, "ON");
            ESP_ERROR_CHECK(ht16k33_ram_write(&dev, all_on));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        ESP_LOGI(tag, "Freeing device descriptor.");
        ESP_ERROR_CHECK(ht16k33_free_desc(&dev));
        memset(&dev, 0, sizeof(i2c_dev_t));
    }
}

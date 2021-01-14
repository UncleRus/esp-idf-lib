#include <esp_err.h>
#include "led_strip_spi.h"
#include "led_strip_spi_sk9822.h"

esp_err_t led_strip_spi_set_pixel(led_strip_spi_t *strip, size_t num, rgb_t color)
{
    int index = (num + 1) * 4;
    ((uint8_t *)strip->buf)[index    ] = LED_STRIP_SPI_FRAME_SK9822_LED_MSB3 + 1; // XXX FIXME brightness control
    ((uint8_t *)strip->buf)[index + 1] = color.b;
    ((uint8_t *)strip->buf)[index + 2] = color.g;
    ((uint8_t *)strip->buf)[index + 3] = color.r;
    return ESP_OK;
}

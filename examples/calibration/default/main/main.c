/*
 * Copyright (c) YYYY YOUR NAME HERE <user@your.dom.ain>
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

#include <stdio.h>
#include <esp_log.h>
#include <calibration.h>

static char *TAG = "calibration_example";

static const calibration_point_t points[] = {
    { .code = 20,  .value = 40 },
    { .code = 10,  .value = 25 },
    { .code = 15,  .value = 31 },
    { .code = 30,  .value = 55 },
    { .code = 100, .value = 250 },
    { .code = 50,  .value = 80 },
};

static const size_t num_points = sizeof(points) / sizeof(calibration_point_t);

static const float test_points[] = {
    0, -40, 15, 16, 30, 300, 31, 35, 40
};

void app_main()
{
    calibration_handle_t c = {0 };
    ESP_ERROR_CHECK(calibration_init(&c, num_points, CALIBRATION_LINEAR));
    ESP_ERROR_CHECK(calibration_add_points(&c, points, num_points));

    ESP_LOGI(TAG, "Calibration points:");
    for (int i = 0; i < c.filled; i++)
        ESP_LOGI(TAG, "Point %d: %.2f ~ %.2f", i, c.points[i].code, c.points[i].value);

    printf("\n");

    ESP_LOGI(TAG, "Test points:");
    for (int i = 0; i < sizeof(test_points) / sizeof(float); i++)
    {
        float v;
        ESP_ERROR_CHECK(calibration_get_value(&c, test_points[i], &v));
        ESP_LOGI(TAG, "Test %d: %.2f ~ %.2f", i, test_points[i], v);
    }
}

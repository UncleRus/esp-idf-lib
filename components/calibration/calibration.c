/**
 * @file calibration.c
 *
 * ESP-IDF library or multi-point calibration
 *
 * Copyright (c) 2023 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "calibration.h"
#include <esp_log.h>
#include <string.h>
#include <inttypes.h>

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "calibration";

static float calc_linear_value(calibration_point_t *p1, calibration_point_t *p2, float code)
{
    float kx = (p2->value - p1->value) / (p2->code - p1->code);
    float sx = kx * p1->code - p1->value;
    return code * kx - sx;
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t calibration_init(calibration_handle_t *handler, size_t count, calibration_method_t type)
{
    CHECK_ARG(handler && count > 1);

    if (type != CALIBRATION_LINEAR)
    {
        ESP_LOGE(TAG, "Only type CALIBRATION_LINEAR is supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (handler->points)
    {
        ESP_LOGE(TAG, "Handler already initialized");
        return ESP_FAIL;
    }

    handler->type = type;
    handler->count = count;
    handler->filled = 0;
    handler->points = calloc(sizeof(calibration_point_t), handler->count);
    if (!handler->points)
    {
        ESP_LOGE(TAG, "Could not allocate %" PRIu32 " bytes", (unsigned long)(sizeof(calibration_point_t) * handler->count));
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t calibration_add_point(calibration_handle_t *handler, float code, float value)
{
    CHECK_ARG(handler && handler->points);

    // FIXME: inefficient
    for (size_t i = 0; i < handler->filled; i++)
        if (handler->points[i].code == code)
        {
            handler->points[i].value = value;
            return ESP_OK;
        }

    if (handler->filled == handler->count)
    {
        ESP_LOGE(TAG, "List of calibration points is full");
        return ESP_ERR_NO_MEM;
    }

    size_t pos;
    for (pos = 0; pos < handler->filled; pos++)
        if (handler->points[pos].code > code)
            break;

    if (pos < handler->filled - 1)
        memmove(handler->points + pos + 1, handler->points + pos, sizeof(calibration_point_t) * (handler->filled - pos));

    handler->points[pos].code = code;
    handler->points[pos].value = value;

    handler->filled++;

    return ESP_OK;
}

esp_err_t calibration_add_points(calibration_handle_t *handler, const calibration_point_t *points, size_t count)
{
    CHECK_ARG(points && count <= handler->count);

    for (size_t i = 0; i < count; i++)
        CHECK(calibration_add_point(handler, points[i].code, points[i].value));

    return ESP_OK;
}

esp_err_t calibration_get_value(calibration_handle_t *handler, float code, float *value)
{
    CHECK_ARG(handler && handler->points && value);

    if (handler->filled < 2)
    {
        ESP_LOGE(TAG, "Not enough calibration points, need at least two");
        return ESP_FAIL;
    }

    // looking for a segment
    size_t pos;
    for (pos = 0; pos < handler->filled; pos++)
    {
        if (handler->points[pos].code == code)
        {
            *value = handler->points[pos].value;
            return ESP_OK;
        }
        if (handler->points[pos].code > code)
            break;
    }
    if (pos > 0)
        pos--;
    if (pos == handler->filled - 1)
        pos--;

    *value = calc_linear_value(handler->points + pos, handler->points + pos + 1, code);

    return ESP_OK;
}

esp_err_t calibration_free(calibration_handle_t *handler)
{
    CHECK_ARG(handler);

    if (handler->points)
    {
        free(handler->points);
        handler->points = NULL;
    }

    return ESP_OK;
}

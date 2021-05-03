/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file framebuffer.c
 *
 * Simple abstraction of RGB framebuffer for ESP-IDF
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <stdlib.h>
#include "framebuffer.h"

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK(x) do { esp_err_t __; if ((__ = (x)) != ESP_OK) return __; } while (0)

static size_t xy(void *ctx, size_t x, size_t y)
{
    framebuffer_t *fb = (framebuffer_t *)ctx;
    return y * fb->width + x;
}

esp_err_t fb_init(framebuffer_t *fb, size_t width, size_t height, fb_render_cb_t render_cb)
{
    CHECK_ARG(fb && width && height && render_cb);

    fb->width = width;
    fb->height = height;
    fb->frame_num = 0;
    fb->last_frame_us = 0;
    fb->render = render_cb;
    fb->internal = NULL;
    fb->mutex = xSemaphoreCreateMutex();
    if (!fb->mutex)
        return ESP_ERR_NO_MEM;
    fb->data = calloc(1, FB_SIZE(fb));
    if (!fb->data)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t fb_free(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    if (fb->data)
        free(fb->data);
    if (fb->mutex)
        vSemaphoreDelete(fb->mutex);

    return ESP_OK;
}

esp_err_t fb_render(framebuffer_t *fb, void *render_ctx)
{
    CHECK_ARG(fb && fb->data && fb->render);

    if (xSemaphoreTake(fb->mutex, 0) != pdTRUE)
        return ESP_ERR_INVALID_STATE;
    CHECK(fb->render(fb, render_ctx));
    xSemaphoreGive(fb->mutex);

    return ESP_OK;
}

esp_err_t fb_set_pixel_rgb(framebuffer_t *fb, size_t x, size_t y, rgb_t color)
{
    CHECK_ARG(fb && fb->data && x < fb->width && y < fb->height);

    fb->data[FB_OFFSET(fb, x, y)] = color;

    return ESP_OK;
}

esp_err_t fb_set_pixel_hsv(framebuffer_t *fb, size_t x, size_t y, hsv_t color)
{
    CHECK_ARG(fb && fb->data && x < fb->width && y < fb->height);

    fb->data[FB_OFFSET(fb, x, y)] = hsv2rgb_rainbow(color);

    return ESP_OK;
}

esp_err_t fb_get_pixel_rgb(framebuffer_t *fb, size_t x, size_t y, rgb_t *color)
{
    CHECK_ARG(color && fb && fb->data && x < fb->width && y < fb->height);

    *color = fb->data[FB_OFFSET(fb, x, y)];

    return ESP_OK;
}

esp_err_t fb_get_pixel_hsv(framebuffer_t *fb, size_t x, size_t y, hsv_t *color)
{
    CHECK_ARG(color && fb && fb->data && x < fb->width && y < fb->height);

    *color = rgb2hsv_approximate(fb->data[FB_OFFSET(fb, x, y)]);

    return ESP_OK;
}

#define WU_WEIGHT(a, b) ((uint8_t)(((a) * (b) + (a) + (b)) >> 8))

esp_err_t fb_set_pixelf_rgb(framebuffer_t *fb, float x, float y, rgb_t color)
{
    CHECK_ARG(fb && fb->data);

    // extract the fractional parts and derive their inverses
    uint8_t xx = (x - (int)x) * 255;
    uint8_t yy = (y - (int)y) * 255;
    uint8_t ix = 255 - xx;
    uint8_t iy = 255 - yy;

    // calculate the intensities for each affected pixel
    uint8_t weights[4] = {
        WU_WEIGHT(ix, iy),
        WU_WEIGHT(xx, iy),
        WU_WEIGHT(ix, yy),
        WU_WEIGHT(xx, yy)
    };
    // multiply the intensities by the colour, and saturating-add them to the pixels
    for (uint8_t i = 0; i < 4; i++)
    {
        int xn = x + (i & 1);
        int yn = y + ((i >> 1) & 1);
        rgb_t clr = { 0 };
        fb_get_pixel_rgb(fb, xn, yn, &clr);
        clr.r = qadd8(clr.r, (color.r * weights[i]) >> 8);
        clr.g = qadd8(clr.g, (color.g * weights[i]) >> 8);
        clr.b = qadd8(clr.b, (color.b * weights[i]) >> 8);
        fb_set_pixel_rgb(fb, xn, yn, clr);
    }

    return ESP_OK;
}

esp_err_t fb_set_pixelf_hsv(framebuffer_t *fb, float x, float y, hsv_t color)
{
    return fb_set_pixelf_rgb(fb, x, y, hsv2rgb_rainbow(color));
}

esp_err_t fb_clear(framebuffer_t *fb)
{
    CHECK_ARG(fb && fb->data);

    memset(fb->data, 0, FB_SIZE(fb));

    return ESP_OK;
}

esp_err_t fb_shift(framebuffer_t *fb, size_t offs, fb_shift_direction_t dir)
{
    CHECK_ARG(fb && fb->data && offs);

    if (((dir == FB_SHIFT_LEFT || dir == FB_SHIFT_RIGHT) && offs >= fb->width)
            || ((dir == FB_SHIFT_UP || dir == FB_SHIFT_DOWN) && offs >= fb->height))
        return ESP_OK;

    switch (dir)
    {
        case FB_SHIFT_LEFT:
            for (size_t row = 0; row < fb->height; row++)
                memmove(fb->data + row * fb->width,
                        fb->data + row * fb->width + offs,
                        sizeof(rgb_t) * (fb->width - offs));
            break;
        case FB_SHIFT_RIGHT:
            for (size_t row = 0; row < fb->height; row++)
                memmove(fb->data + row * fb->width + offs,
                        fb->data + row * fb->width,
                        sizeof(rgb_t) * (fb->width - offs));
            break;
        case FB_SHIFT_UP:
            memmove(fb->data + offs * fb->width,
                    fb->data,
                    FB_SIZE(fb) - offs * fb->width * sizeof(rgb_t));
            break;
        case FB_SHIFT_DOWN:
            memmove(fb->data,
                    fb->data + offs * fb->width,
                    FB_SIZE(fb) - offs * fb->width * sizeof(rgb_t));
            break;
    }

    return ESP_OK;
}

esp_err_t fb_fade(framebuffer_t *fb, uint8_t scale)
{
    CHECK_ARG(fb && fb->data);

    for (size_t i = 0; i < fb->width * fb->height; i++)
        fb->data[i] = rgb_fade(fb->data[i], scale);

    return ESP_OK;
}

esp_err_t fb_blur2d(framebuffer_t *fb, fract8 amount)
{
    CHECK_ARG(fb && fb->data);

    blur2d(fb->data, fb->width, fb->height, amount, xy, fb);

    return ESP_OK;
}

esp_err_t fb_begin(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    if (xSemaphoreTake(fb->mutex, 0) != pdTRUE)
        return ESP_ERR_INVALID_STATE;

    return ESP_OK;
}

esp_err_t fb_end(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    fb->frame_num++;
    fb->last_frame_us = esp_timer_get_time();
    xSemaphoreGive(fb->mutex);

    return ESP_OK;
}

#include <stdlib.h>
#include "framebuffer.h"

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

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
    fb->data = calloc(FB_SIZE(fb), 1);
    if (!fb->data)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t fb_free(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    if (fb->data)
        free(fb->data);

    return ESP_OK;
}

esp_err_t fb_render(framebuffer_t *fb, void *render_ctx)
{
    CHECK_ARG(fb && fb->data && fb->render);

    if (fb->busy)
        return ESP_ERR_INVALID_STATE;

    return fb->render(fb, render_ctx);
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

    if (fb->busy)
        return ESP_ERR_INVALID_STATE;

    fb->busy = true;

    return ESP_OK;
}

esp_err_t fb_end(framebuffer_t *fb)
{
    CHECK_ARG(fb);

    fb->frame_num++;
    fb->last_frame_us = esp_timer_get_time();
    fb->busy = false;

    return ESP_OK;
}

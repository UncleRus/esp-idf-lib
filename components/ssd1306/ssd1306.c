/**
 * @file ssd1306.h
 * @defgroup ssd1306 ssd1306
 * @{
 *
 * ESP-IDF driver for SSD1306/SH1106 OLED displays.
 *
 * ported from esp-open-rtos
 *
 * Copyright (c) 2016, 2021 urx (https://github.com/urx),
 *                          Ruslan V. Uss (https://github.com/UncleRus)
 *                          Zaltora (https://github.com/Zaltora)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include "ssd1306.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_FREQ_HZ 400000 // 400kHz
#define SPI_FREQ_HZ 8000000 // 8MHz

static const char *TAG = "ssd1306";

#define I2C_CMD  0x00
#define I2C_DATA 0x40

#define SPI_CMD  0
#define SPI_DATA 1

/* SSD1306 commands */
#define SSD1306_SET_MEM_ADDR_MODE    (0x20)

#define SSD1306_SET_COL_ADDR         (0x21)
#define SSD1306_SET_PAGE_ADDR        (0x22)
#define SSD1306_SET_DISP_START_LINE  (0x40)
#define SSD1306_SET_CONTRAST         (0x81)
#define SSD1306_SET_SEGMENT_REMAP0   (0xA0)
#define SSD1306_SET_SEGMENT_REMAP1   (0xA1)
#define SSD1306_SET_ENTIRE_DISP_ON   (0xA5)
#define SSD1306_SET_ENTIRE_DISP_OFF  (0xA4)
#define SSD1306_SET_INVERSION_OFF    (0xA6)
#define SSD1306_SET_INVERSION_ON     (0xA7)

#define SSD1306_SET_MUX_RATIO        (0xA8)
#define SSD1306_MUX_RATIO_MASK       (0x3F)
#define SSD1306_SET_DISPLAY_OFF      (0xAE)
#define SSD1306_SET_DISPLAY_ON       (0xAF)
#define SSD1306_SET_SCAN_DIR_FWD     (0xC0)
#define SSD1306_SET_SCAN_DIR_BWD     (0xC8)
#define SSD1306_SET_DISPLAY_OFFSET   (0xD3)
#define SSD1306_SET_OSC_FREQ         (0xD5)
#define SSD1306_SET_PRE_CHRG_PER     (0xD9)

#define SSD1306_SET_COM_PINS_HW_CFG  (0xDA)
#define SSD1306_COM_PINS_HW_CFG_MASK (0x32)
#define SSD1306_SEQ_COM_PINS_CFG     (0x02)
#define SSD1306_ALT_COM_PINS_CFG     (0x12)
#define SSD1306_COM_LR_REMAP_OFF     (0x02)
#define SSD1306_COM_LR_REMAP_ON      (0x22)

#define SSD1306_SET_DESEL_LVL        (0xDB)
#define SSD1306_SET_NOP              (0xE3)

#define SSD1306_SET_CHARGE_PUMP      (0x8D)
#define SSD1306_CHARGE_PUMP_EN       (0x14)
#define SSD1306_CHARGE_PUMP_DIS      (0x10)

#define SSD1306_SCROLL_HOR_LEFT      (0x27)
#define SSD1306_SCROLL_HOR_RIGHT     (0x26)
#define SSD1306_SCROLL_HOR_VER_LEFT  (0x2A)
#define SSD1306_SCROLL_HOR_VER_RIGHT (0x29)
#define SSD1306_SCROLL_ENABLE        (0x2F)
#define SSD1306_SCROLL_DISABLE       (0x2E)

#define SH1106_SET_CHARGE_PUMP       (0xAD)
#define SH1106_CHARGE_PUMP_EN        (0x8B)
#define SH1106_CHARGE_PUMP_DIS       (0x8A)
#define SH1106_CHARGE_PUMP_VALUE     (0x30)

#define SH1106_SET_PAGE_ADDRESS      (0xB0)
#define SH1106_SET_LOW_COL_ADDR      (0x00)
#define SH1106_SET_HIGH_COL_ADDR     (0x10)

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#if CONFIG_SSD1306_PROTOCOL_SPI4 || CONFIG_SSD1306_PROTOCOL_SPI3
#define SPI_CHECK(dev, x) do { esp_err_t __; if ((__ = x) != ESP_OK) { spi_device_release_bus(dev); return __; }} while (0)
#endif

#define FRAMEBUF_SIZE(dev) ((dev)->width * (dev)->height / 8)

#if CONFIG_SSD1306_PROTOCOL_SPI4 || CONFIG_SSD1306_PROTOCOL_SPI3
static esp_err_t spi_send(ssd1306_t *dev, uint32_t dc, const uint8_t *data, size_t length)
{
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));

    trans.length = length * 8;
    trans.tx_buffer = data;

#if CONFIG_SSD1306_PROTOCOL_SPI3
    trans.cmd = dc & 1;
#endif
#if CONFIG_SSD1306_PROTOCOL_SPI4
    gpio_set_level(dev->dc_gpio, dc);
#endif

    return spi_device_polling_transmit(dev->spi_dev, &trans);
}
#endif

static esp_err_t send_cmd(ssd1306_t *dev, uint8_t cmd)
{
    ESP_LOGD(TAG, "Command: 0x%02x", cmd);
#if CONFIG_SSD1306_PROTOCOL_I2C
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, &cmd, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
#endif
#if CONFIG_SSD1306_PROTOCOL_SPI4 || CONFIG_SSD1306_PROTOCOL_SPI3
    CHECK(spi_device_acquire_bus(dev->spi_dev, portMAX_DELAY));
    CHECK(spi_send(dev, SPI_CMD, &cmd, 1));
    spi_device_release_bus(dev->spi_dev);
#endif
    return ESP_OK;
}

static esp_err_t send_cmd_arg(ssd1306_t *dev, uint8_t cmd, uint8_t arg)
{
    ESP_LOGD(TAG, "Command: 0x%02x, arg: 0x%02x", cmd, arg);
#if CONFIG_SSD1306_PROTOCOL_I2C
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, &cmd, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, &arg, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
#endif
#if CONFIG_SSD1306_PROTOCOL_SPI4 || CONFIG_SSD1306_PROTOCOL_SPI3
    CHECK(spi_device_acquire_bus(dev->spi_dev, portMAX_DELAY));
    CHECK(spi_send(dev, SPI_CMD, &cmd, 1));
    CHECK(spi_send(dev, SPI_CMD, &arg, 1));
    spi_device_release_bus(dev->spi_dev);
#endif
    return ESP_OK;
}

static esp_err_t send_cmd_arg2(ssd1306_t *dev, uint8_t cmd, uint8_t arg1, uint8_t arg2)
{
    ESP_LOGD(TAG, "Command: 0x%02x, args: 0x%02x, 0x%02x", cmd, arg1, arg2);
#if CONFIG_SSD1306_PROTOCOL_I2C
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, &cmd, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, &arg1, 1));
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, &arg2, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
#endif
#if CONFIG_SSD1306_PROTOCOL_SPI4 || CONFIG_SSD1306_PROTOCOL_SPI3
    CHECK(spi_device_acquire_bus(dev->spi_dev, portMAX_DELAY));
    CHECK(spi_send(dev, SPI_CMD, &cmd, 1));
    CHECK(spi_send(dev, SPI_CMD, &arg1, 1));
    CHECK(spi_send(dev, SPI_CMD, &arg2, 1));
    spi_device_release_bus(dev->spi_dev);
#endif
    return ESP_OK;
}

static int sh1106_go_coordinate(ssd1306_t *dev, uint8_t x, uint8_t y)
{
    //offset : panel is 128 ; RAM is 132 for sh1106
    x += 2;
    uint8_t cmd[3] = {
            SH1106_SET_PAGE_ADDRESS + y,         // Set row
            SH1106_SET_LOW_COL_ADDR | (x & 0xf), // Set lower column address
            SH1106_SET_HIGH_COL_ADDR | (x >> 4)  // Set higher column address
    };

#if CONFIG_SSD1306_PROTOCOL_I2C
    CHECK(i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, cmd, 1));
    CHECK(i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, cmd + 1, 1));
    CHECK(i2c_dev_write_reg(&dev->i2c_dev, I2C_CMD, cmd + 2, 1));
#endif
#if CONFIG_SSD1306_PROTOCOL_SPI4 || CONFIG_SSD1306_PROTOCOL_SPI3
    CHECK(spi_send(dev, SPI_CMD, cmd, 1));
    CHECK(spi_send(dev, SPI_CMD, cmd + 1, 1));
    CHECK(spi_send(dev, SPI_CMD, cmd + 2, 1));
#endif

    return ESP_OK;
}

////////////////////////////////////////////////////////////////////////////////

#if CONFIG_SSD1306_PROTOCOL_I2C
esp_err_t ssd1306_init_desc(ssd1306_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev && (addr == SSD1306_I2C_ADDR0 || addr == SSD1306_I2C_ADDR1));

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}
#endif

#if CONFIG_SSD1306_PROTOCOL_SPI4
esp_err_t ssd1306_init_desc(ssd1306_t *dev, spi_host_device_t host, gpio_num_t cs_gpio, gpio_num_t dc_gpio)
{
    CHECK_ARG(dev);

    dev->dc_gpio = dc_gpio;
    CHECK(gpio_set_direction(dc_gpio, GPIO_MODE_OUTPUT));

    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = cs_gpio;
    dev->spi_cfg.clock_speed_hz = SPI_FREQ_HZ;
    dev->spi_cfg.mode = 0;
    dev->spi_cfg.queue_size = 1;
    dev->spi_cfg.flags = SPI_DEVICE_NO_DUMMY;

    return spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
}
#endif

#if CONFIG_SSD1306_PROTOCOL_SPI3
esp_err_t ssd1306_init_desc(ssd1306_t *dev, spi_host_device_t host, gpio_num_t cs_gpio)
{
    CHECK_ARG(dev);

    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = cs_gpio;
    dev->spi_cfg.clock_speed_hz = SPI_FREQ_HZ;
    dev->spi_cfg.mode = 0;
    dev->spi_cfg.queue_size = 1;
    dev->spi_cfg.command_bits = 1;
    dev->spi_cfg.flags = SPI_DEVICE_NO_DUMMY;

    return spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
}
#endif

esp_err_t ssd1306_free_desc(ssd1306_t *dev)
{
    CHECK_ARG(dev);

#if CONFIG_SSD1306_PROTOCOL_I2C
    return i2c_dev_delete_mutex(&dev->i2c_dev);
#endif
#if CONFIG_SSD1306_PROTOCOL_SPI4 || CONFIG_SSD1306_PROTOCOL_SPI3
    return spi_bus_remove_device(dev->spi_dev);
#endif
}

/* Perform default init routine according
 * to SSD1306 datasheet from adafruit.com */
esp_err_t ssd1306_init(ssd1306_t *dev)
{
    CHECK_ARG(dev && (dev->chip == SSD1306_CHIP || dev->chip == SH1106_CHIP));

    uint8_t pin_cfg;
    switch (dev->height)
    {
        case 16:
        case 32:
            pin_cfg = 0x02;
            break;
        case 64:
            pin_cfg = 0x12;
            break;
        default:
            ESP_LOGE(TAG, "Unsupported screen height: %d", dev->height);
            return ESP_ERR_NOT_SUPPORTED;
    }

    // allocate framebuffer memory
    dev->fb = calloc(1, FRAMEBUF_SIZE(dev));
    if (!dev->fb)
        return ESP_ERR_NO_MEM;

    CHECK(ssd1306_display_on(dev, false));
    CHECK(ssd1306_set_osc_freq(dev, 0x80));
    CHECK(ssd1306_set_charge_pump_enabled(dev, true));
    if (dev->chip == SH1106_CHIP)
        CHECK(sh1106_set_charge_pump_voltage(dev, SH1106_VOLTAGE_74));
    CHECK(ssd1306_set_mux_ratio(dev, dev->height - 1));
    CHECK(ssd1306_set_display_offset(dev, 0x0));
    CHECK(ssd1306_set_display_start_line(dev, 0x0));
    if (dev->chip == SSD1306_CHIP)
        CHECK(ssd1306_set_mem_addr_mode(dev, SSD1306_ADDR_MODE_HORIZONTAL));
    CHECK(ssd1306_set_segment_remapping_enabled(dev, false));
    CHECK(ssd1306_set_scan_direction_fwd(dev, true));
    CHECK(ssd1306_set_com_pin_hw_config(dev, pin_cfg));
    CHECK(ssd1306_set_contrast(dev, 0x9f));
    CHECK(ssd1306_set_precharge_period(dev, 0xf1));
    CHECK(ssd1306_set_deseltct_lvl(dev, 0x40));
    CHECK(ssd1306_set_inversion(dev, false));
    CHECK(ssd1306_set_display_mode(dev, SSD1306_MODE_NORMAL));
    CHECK(ssd1306_display_on(dev, true));

    return ESP_OK;
}

esp_err_t ssd1306_done(ssd1306_t *dev)
{
    CHECK_ARG(dev && dev->fb);

    free(dev->fb);

    return ESP_OK;
}

esp_err_t ssd1306_flush(ssd1306_t *dev)
{
    CHECK_ARG(dev && dev->fb);

    size_t len = FRAMEBUF_SIZE(dev);
    if (dev->chip == SSD1306_CHIP)
    {
        ssd1306_set_column_addr(dev, 0, dev->width - 1);
        ssd1306_set_page_addr(dev, 0, dev->height / 8 - 1);
    }

#if CONFIG_SSD1306_PROTOCOL_I2C
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    for (size_t i = 0; i < len; i += 16)
    {
        if (dev->chip == SH1106_CHIP && i % dev->width == 0)
            I2C_DEV_CHECK(&dev->i2c_dev, sh1106_go_coordinate(dev, 0, i / dev->width));
        I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, I2C_DATA, dev->fb + i, 16));
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
#endif
#if CONFIG_SSD1306_PROTOCOL_SPI4 || CONFIG_SSD1306_PROTOCOL_SPI3
    CHECK(spi_device_acquire_bus(dev->spi_dev, portMAX_DELAY));
    if (dev->chip == SSD1306_CHIP)
        SPI_CHECK(dev->spi_dev, spi_send(dev, SPI_DATA, dev->fb, len));
    else
        for (size_t i = 0; i < dev->height / 8; i++)
        {
            SPI_CHECK(dev->spi_dev, sh1106_go_coordinate(dev, 0, i));
            SPI_CHECK(dev->spi_dev, spi_send(dev, SPI_DATA, dev->fb + dev->width * i, dev->width));
        }
    spi_device_release_bus(dev->spi_dev);
#endif

    return ESP_OK;
}

esp_err_t ssd1306_display_on(ssd1306_t *dev, bool on)
{
    CHECK_ARG(dev);

    return send_cmd(dev, on ? SSD1306_SET_DISPLAY_ON : SSD1306_SET_DISPLAY_OFF);
}

esp_err_t ssd1306_set_display_start_line(ssd1306_t *dev, uint8_t start)
{
    CHECK_ARG(dev && start <= 63);

    return send_cmd(dev, SSD1306_SET_DISP_START_LINE | start);
}

esp_err_t ssd1306_set_display_offset(ssd1306_t *dev, uint8_t offset)
{
    CHECK_ARG(dev && offset <= 63);

    return send_cmd_arg(dev, SSD1306_SET_DISPLAY_OFFSET, offset);
}

esp_err_t sh1106_set_charge_pump_voltage(ssd1306_t *dev, sh1106_voltage_t select)
{
    CHECK_ARG(dev && dev->chip == SH1106_CHIP);

    return send_cmd(dev, select | SH1106_CHARGE_PUMP_VALUE);
}

esp_err_t ssd1306_set_charge_pump_enabled(ssd1306_t *dev, bool enabled)
{
    CHECK_ARG(dev);

    if (dev->chip == SSD1306_CHIP)
        return send_cmd_arg(dev, SSD1306_SET_CHARGE_PUMP,
                enabled ? SSD1306_CHARGE_PUMP_EN : SSD1306_CHARGE_PUMP_DIS);
    else
        return send_cmd_arg(dev, SH1106_SET_CHARGE_PUMP,
                enabled ? SH1106_CHARGE_PUMP_EN : SH1106_CHARGE_PUMP_DIS);
}

esp_err_t ssd1306_set_mem_addr_mode(ssd1306_t *dev, ssd1306_mem_addr_mode_t mode)
{
    CHECK_ARG(dev);
    if (dev->chip == SH1106_CHIP)
        return ESP_ERR_NOT_SUPPORTED;

    return send_cmd_arg(dev, SSD1306_SET_MEM_ADDR_MODE, mode);
}

esp_err_t ssd1306_set_segment_remapping_enabled(ssd1306_t *dev, bool on)
{
    CHECK_ARG(dev);

    return send_cmd(dev, on ? SSD1306_SET_SEGMENT_REMAP1 : SSD1306_SET_SEGMENT_REMAP0);
}

esp_err_t ssd1306_set_scan_direction_fwd(ssd1306_t *dev, bool fwd)
{
    CHECK_ARG(dev);

    return send_cmd(dev, fwd ? SSD1306_SET_SCAN_DIR_FWD : SSD1306_SET_SCAN_DIR_BWD);
}

esp_err_t ssd1306_set_com_pin_hw_config(ssd1306_t *dev, uint8_t config)
{
    CHECK_ARG(dev);

    return send_cmd_arg(dev, SSD1306_SET_COM_PINS_HW_CFG, config & SSD1306_COM_PINS_HW_CFG_MASK);
}

esp_err_t ssd1306_set_contrast(ssd1306_t *dev, uint8_t contrast)
{
    CHECK_ARG(dev);

    return send_cmd_arg(dev, SSD1306_SET_CONTRAST, contrast);
}

esp_err_t ssd1306_set_inversion(ssd1306_t *dev, bool on)
{
    CHECK_ARG(dev);

    return send_cmd(dev, on ? SSD1306_SET_INVERSION_ON : SSD1306_SET_INVERSION_OFF);
}

esp_err_t ssd1306_set_osc_freq(ssd1306_t *dev, uint8_t osc_freq)
{
    CHECK_ARG(dev);

    return send_cmd_arg(dev, SSD1306_SET_OSC_FREQ, osc_freq);
}

esp_err_t ssd1306_set_mux_ratio(ssd1306_t *dev, uint8_t ratio)
{
    CHECK_ARG(dev && ratio >= 15 && ratio <= 63);

    return send_cmd_arg(dev, SSD1306_SET_MUX_RATIO, ratio);
}

esp_err_t ssd1306_set_column_addr(ssd1306_t *dev, uint8_t start, uint8_t stop)
{
    CHECK_ARG(dev);

    return send_cmd_arg2(dev, SSD1306_SET_COL_ADDR, start, stop);
}

esp_err_t ssd1306_set_page_addr(ssd1306_t *dev, uint8_t start, uint8_t stop)
{
    CHECK_ARG(dev);

    return send_cmd_arg2(dev, SSD1306_SET_PAGE_ADDR, start, stop);
}

esp_err_t ssd1306_set_precharge_period(ssd1306_t *dev, uint8_t prchrg)
{
    CHECK_ARG(dev);

    return send_cmd_arg(dev, SSD1306_SET_PRE_CHRG_PER, prchrg);
}

esp_err_t ssd1306_set_deseltct_lvl(ssd1306_t *dev, uint8_t lvl)
{
    CHECK_ARG(dev);

    return send_cmd_arg(dev, SSD1306_SET_DESEL_LVL, lvl);
}

esp_err_t ssd1306_set_display_mode(ssd1306_t *dev, ssd1306_display_mode_t mode)
{
    CHECK_ARG(dev);

    return send_cmd(dev, mode == SSD1306_MODE_FILL ? SSD1306_SET_ENTIRE_DISP_ON : SSD1306_SET_ENTIRE_DISP_OFF);
}

/* one byte of xbm - 8 dots in line of picture source
 * one byte of fb - 8 rows for 1 column of screen
 */
esp_err_t ssd1306_load_xbm(ssd1306_t *dev, uint8_t *xbm)
{
    CHECK_ARG(dev && dev->fb && xbm);

    uint8_t bit = 0;

    int row = 0;
    int column = 0;
    for (row = 0; row < dev->height; row++)
    {
        for (column = 0; column < dev->width / 8; column++)
        {
            uint16_t xbm_offset = row * 16 + column;
            for (bit = 0; bit < 8; bit++)
            {
                if (*(xbm + xbm_offset) & 1 << bit)
                    *(dev->fb + dev->width * (row / 8) + column * 8 + bit) |= 1 << row % 8;
            }
        }
    }

    return ESP_OK;
}

esp_err_t ssd1306_set_pixel(ssd1306_t *dev, uint8_t x, uint8_t y, ssd1306_color_t color)
{
    CHECK_ARG(dev && dev->fb && x < dev->width && y < dev->height);

    size_t index = x + (y / 8) * dev->width;
    switch (color)
    {
        case OLED_COLOR_WHITE:
            dev->fb[index] |= (1 << (y & 7));
            break;
        case OLED_COLOR_BLACK:
            dev->fb[index] &= ~(1 << (y & 7));
            break;
        case OLED_COLOR_INVERT:
            dev->fb[index] ^= (1 << (y & 7));
            break;
        default:
            break;
    }

    return ESP_OK;
}

esp_err_t ssd1306_draw_hline(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w, ssd1306_color_t color)
{
    CHECK_ARG(dev && dev->fb && w && x < dev->width && y < dev->height);

    // boundary check
    if (x + w > dev->width)
        w = dev->width - x;

    uint8_t t = w;
    size_t index = x + (y / 8) * dev->width;
    uint8_t mask = 1 << (y & 7);
    switch (color)
    {
        case OLED_COLOR_WHITE:
            while (t--)
            {
                dev->fb[index] |= mask;
                ++index;
            }
            break;
        case OLED_COLOR_BLACK:
            mask = ~mask;
            while (t--)
            {
                dev->fb[index] &= mask;
                ++index;
            }
            break;
        case OLED_COLOR_INVERT:
            while (t--)
            {
                dev->fb[index] ^= mask;
                ++index;
            }
            break;
        default:
            break;
    }

    return ESP_OK;
}

esp_err_t ssd1306_draw_vline(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t h, ssd1306_color_t color)
{
    CHECK_ARG(dev && dev->fb && h && x < dev->width && y < dev->height);

    // boundary check
    if (y + h > dev->height)
        h = dev->height - y;

    uint8_t mask;
    uint8_t t = h;
    size_t index = x + (y / 8) * dev->width;
    uint8_t mod = y & 7;
    if (mod) // partial line that does not fit into byte at top
    {
        mod = 8 - mod;
        static const uint8_t premask[8] = { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        mask = premask[mod];
        if (t < mod)
            mask &= (0xFF >> (mod - t));
        switch (color)
        {
            case OLED_COLOR_WHITE:
                dev->fb[index] |= mask;
                break;
            case OLED_COLOR_BLACK:
                dev->fb[index] &= ~mask;
                break;
            case OLED_COLOR_INVERT:
                dev->fb[index] ^= mask;
                break;
            default:
                break;
        }

        if (t < mod)
            return 0;
        t -= mod;
        index += dev->width;
    }
    if (t >= 8) // byte aligned line at middle
    {
        switch (color)
        {
            case OLED_COLOR_WHITE:
                do
                {
                    dev->fb[index] = 0xff;
                    index += dev->width;
                    t -= 8;
                } while (t >= 8);
                break;
            case OLED_COLOR_BLACK:
                do
                {
                    dev->fb[index] = 0x00;
                    index += dev->width;
                    t -= 8;
                } while (t >= 8);
                break;
            case OLED_COLOR_INVERT:
                do
                {
                    dev->fb[index] = ~dev->fb[index];
                    index += dev->width;
                    t -= 8;
                } while (t >= 8);
                break;
            default:
                break;
        }
    }
    if (t) // partial line at bottom
    {
        mod = t & 7;
        static const uint8_t postmask[8] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        mask = postmask[mod];
        switch (color)
        {
            case OLED_COLOR_WHITE:
                dev->fb[index] |= mask;
                break;
            case OLED_COLOR_BLACK:
                dev->fb[index] &= ~mask;
                break;
            case OLED_COLOR_INVERT:
                dev->fb[index] ^= mask;
                break;
            default:
                break;
        }
    }

    return ESP_OK;
}

esp_err_t ssd1306_clear(ssd1306_t *dev)
{
    CHECK_ARG(dev && dev->fb);

    memset(dev->fb, 0, FRAMEBUF_SIZE(dev));

    return ESP_OK;
}

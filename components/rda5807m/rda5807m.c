/**
 * @file rda5807m.c
 *
 * ESP-IDF driver for single-chip broadcast FM radio tuner RDA5807M
 *
 * Copyright (C) 2018 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_idf_lib_helpers.h>
#include "rda5807m.h"
#include <esp_log.h>
#include <string.h>
#include <esp_err.h>

#define I2C_FREQ_HZ 400000 // 400kHz

#define I2C_ADDR_SEQ 0x10
#define I2C_ADDR_IDX 0x11

// Registers
#define REG_CHIP_ID 0x00
#define REG_CTRL    0x02
#define REG_CHAN    0x03
#define REG_R4      0x04
#define REG_VOL     0x05
#define REG_R7      0x07
#define REG_RA      0x0a
#define REG_RB      0x0b
#define REG_RDSA    0x0c
#define REG_RDSB    0x0d
#define REG_RDSC    0x0c
#define REG_RDSD    0x0d

// Bits
#define BIT_CTRL_ENABLE      0
#define BIT_CTRL_SOFT_RESET  1
#define BIT_CTRL_NEW_METHOD  2
#define BIT_CTRL_RDS_EN      3
#define BIT_CTRL_CLK_MODE    4
#define BIT_CTRL_SKMODE      7
#define BIT_CTRL_SEEK        8
#define BIT_CTRL_SEEKUP      9
#define BIT_CTRL_RCLK_DIRECT 10
#define BIT_CTRL_RCLK_NC     11
#define BIT_CTRL_BASS        12
#define BIT_CTRL_MONO        13
#define BIT_CTRL_DMUTE       14
#define BIT_CTRL_DHIZ        15

#define BIT_CHAN_SPACE 0
#define BIT_CHAN_BAND  2
#define BIT_CHAN_TUNE  4
#define BIT_CHAN_CHAN  6

#define BIT_R4_AFCD        8
#define BIT_R4_SOFTMUTE_EN 9
#define BIT_R4_DE          11

#define BIT_VOL_VOLUME   0
#define BIT_VOL_SEEKTH   8
#define BIT_VOL_INT_MODE 15

#define BIT_R7_SOFTBLEND_EN 1
#define BIT_R7_50M          9
#define BIT_R7_TH_SOFTBLEND 10

#define BIT_RA_READCHAN 0
#define BIT_RA_ST       10
#define BIT_RA_BLK_E    11
#define BIT_RA_RDSS     12
#define BIT_RA_SF       13
#define BIT_RA_STC      14
#define BIT_RA_RDSR     15

#define BIT_RB_BLERB    0
#define BIT_RB_BLERA    2
#define BIT_RB_ABCD_E   4
#define BIT_RB_FM_READY 7
#define BIT_RB_FM_ST    8
#define BIT_RB_RSSI     9

// Masks
#define MASK_CHAN_SPACE 0x0003
#define MASK_CHAN_BAND  0x000c
#define MASK_CHAN_CHAN  0xffc0  // high 10 bits

#define MASK_VOL_VOLUME  0x000f
#define MASK_VOL_SEEKTH  0x0f00

#define MASK_RA_READCHAN 0x3ff

#define MAX_CHAN 0x3ff

#define BV(x) (1 << (x))
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const uint8_t spacings[] = {
    [RDA5807M_CHAN_SPACE_100] = 100,
    [RDA5807M_CHAN_SPACE_200] = 200,
    [RDA5807M_CHAN_SPACE_50]  = 50,
    [RDA5807M_CHAN_SPACE_25]  = 25
};

typedef struct
{
    uint32_t lower;
    uint32_t upper;
} band_limit_t;

static const band_limit_t band_limits[] = {
    [RDA5807M_BAND_87_108] = {87000, 108000},
    [RDA5807M_BAND_76_91]  = {76000, 91000},
    [RDA5807M_BAND_76_108] = {76000, 108000},
    [RDA5807M_BAND_65_76]  = {65000, 76000},
    [RDA5807M_BAND_50_76]  = {50000, 76000}
};

static const char *TAG = "rda5807m";

static inline esp_err_t read_register_nolock(rda5807m_t *dev, uint8_t reg, uint16_t *val)
{
    dev->i2c_dev.addr = I2C_ADDR_IDX;
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, reg, val, 2));

    *val = (*val >> 8) | (*val << 8);

    return ESP_OK;
}

static inline esp_err_t write_register_nolock(rda5807m_t *dev, uint8_t reg, uint16_t val)
{
    uint16_t v = (val >> 8) | (val << 8);

    dev->i2c_dev.addr = I2C_ADDR_IDX;
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &v, 2);
}

static esp_err_t read_register(rda5807m_t *dev, uint8_t reg, uint16_t *val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_register_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t update_register_nolock(rda5807m_t *dev, uint8_t reg, uint16_t mask, uint16_t val)
{
    uint16_t old;

    CHECK(read_register_nolock(dev, reg, &old));
    CHECK(write_register_nolock(dev, reg, (old & ~mask) | val));

    return ESP_OK;
}

static esp_err_t update_register(rda5807m_t *dev, uint8_t reg, uint16_t mask, uint16_t val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, update_register_nolock(dev, reg, mask, val));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_registers_bulk(rda5807m_t *dev, uint16_t *val, uint8_t count)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    dev->i2c_dev.addr = I2C_ADDR_SEQ;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read(&dev->i2c_dev, NULL, 0, val, count * 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    for (uint8_t i = 0; i < count; i++)
        val[i] = (val[i] >> 8) | (val[i] << 8);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t rda5807m_init_desc(rda5807m_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = I2C_ADDR_IDX;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t rda5807m_free_desc(rda5807m_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t rda5807m_init(rda5807m_t *dev, rda5807m_clock_freq_t clock_freq)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    // reset
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_nolock(dev, REG_CTRL, BV(BIT_CTRL_SOFT_RESET) | BV(BIT_CTRL_ENABLE)));

    I2C_DEV_CHECK(&dev->i2c_dev, write_register_nolock(dev, REG_CTRL,
            ((clock_freq & 7) << BIT_CTRL_CLK_MODE) | // Set clock mode
            BV(BIT_CTRL_DHIZ) |                       // Enable audio output
            BV(BIT_CTRL_DMUTE) |                      // Disable mute
            BV(BIT_CTRL_RDS_EN) |                     // Enable RDS
            BV(BIT_CTRL_ENABLE)                       // Enable chip
    ));
    // De-emphasis = 50us
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_nolock(dev, REG_R4, BV(BIT_R4_DE)));
    // Set volume = 0, Seek threshold = ~32 dB SNR, INT_MODE = 1 (?)
    I2C_DEV_CHECK(&dev->i2c_dev, write_register_nolock(dev, REG_VOL, BV(BIT_VOL_INT_MODE) | (8 << BIT_VOL_SEEKTH)));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->band = RDA5807M_BAND_87_108;
    dev->spacing = RDA5807M_CHAN_SPACE_100;

    ESP_LOGI(TAG, "Device initialized");

    return ESP_OK;
}

esp_err_t rda5807m_get_state(rda5807m_t *dev, rda5807m_state_t *state)
{
    CHECK_ARG(dev);
    CHECK_ARG(state);

    uint16_t r[6];
    uint16_t ctrl;
    CHECK(read_registers_bulk(dev, r, 6));
    CHECK(read_register(dev, REG_CTRL, &ctrl));

    if (ctrl & BIT_CTRL_SEEK) state->seek_status = RDA5807M_SEEK_STARTED;
    else if (r[0] & BIT_RA_SF) state->seek_status = RDA5807M_SEEK_FAILED;
    else if (r[0] & BIT_RA_STC) state->seek_status = RDA5807M_SEEK_COMPLETE;
    else state->seek_status = RDA5807M_SEEK_NONE;

    state->frequency = (r[0] & MASK_RA_READCHAN) * spacings[dev->spacing] + band_limits[dev->band].lower;
    state->stereo = (r[0] & BIT_RA_ST) != 0;
    state->station = (r[1] & BIT_RB_FM_ST) != 0;
    state->rds_ready = (r[0] & BIT_RA_RDSR) != 0;
    state->rssi = r[1] >> BIT_RB_RSSI;

    memcpy(state->rds, &r[2], 8);

    return ESP_OK;
}

esp_err_t rda5807m_get_volume(rda5807m_t *dev, uint8_t *vol)
{
    CHECK_ARG(dev);
    CHECK_ARG(vol);

    uint16_t v;
    CHECK(read_register(dev, REG_VOL, &v));
    *vol = v & MASK_VOL_VOLUME;

    return ESP_OK;
}

esp_err_t rda5807m_set_volume(rda5807m_t *dev, uint8_t vol)
{
    CHECK_ARG(dev);
    CHECK_ARG(vol <= RDA5807M_VOL_MAX);

    CHECK(update_register(dev, REG_VOL, MASK_VOL_VOLUME, vol));

    ESP_LOGI(TAG, "Volume set to %d", vol);

    return ESP_OK;
}

esp_err_t rda5807m_get_mute(rda5807m_t *dev, bool *mute)
{
    CHECK_ARG(dev);
    CHECK_ARG(mute);

    uint16_t v;
    CHECK(read_register(dev, REG_CTRL, &v));
    *mute = !(v & BV(BIT_CTRL_DMUTE));

    return ESP_OK;
}

esp_err_t rda5807m_set_mute(rda5807m_t *dev, bool mute)
{
    CHECK_ARG(dev);

    CHECK(update_register(dev, REG_CTRL, BV(BIT_CTRL_DMUTE), mute ? 0 : BV(BIT_CTRL_DMUTE)));

    ESP_LOGI(TAG, "Mute %s", mute ? "enabled" : "disabled");

    return ESP_OK;
}

esp_err_t rda5807m_get_softmute(rda5807m_t *dev, bool *softmute)
{
    CHECK_ARG(dev);
    CHECK_ARG(softmute);

    uint16_t v;
    CHECK(read_register(dev, REG_R4, &v));
    *softmute = v & BV(BIT_R4_SOFTMUTE_EN);

    return ESP_OK;
}

esp_err_t rda5807m_set_softmute(rda5807m_t *dev, bool softmute)
{
    CHECK_ARG(dev);

    CHECK(update_register(dev, REG_R4, BV(BIT_R4_SOFTMUTE_EN), softmute ? BV(BIT_R4_SOFTMUTE_EN) : 0));

    ESP_LOGI(TAG, "Softmute %s", softmute ? "enabled" : "disabled");

    return ESP_OK;
}

esp_err_t rda5807m_get_bass_boost(rda5807m_t *dev, bool *bass_boost)
{
    CHECK_ARG(dev);
    CHECK_ARG(bass_boost);

    uint16_t v;
    CHECK(read_register(dev, REG_CTRL, &v));
    *bass_boost = v & BV(BIT_CTRL_BASS);

    return ESP_OK;
}

esp_err_t rda5807m_set_bass_boost(rda5807m_t *dev, bool bass_boost)
{
    CHECK_ARG(dev);

    CHECK(update_register(dev, REG_CTRL, BV(BIT_CTRL_BASS), bass_boost ? BV(BIT_CTRL_BASS) : 0));

    ESP_LOGI(TAG, "Bass-boost %s", bass_boost ? "enabled" : "disabled");

    return ESP_OK;
}

esp_err_t rda5807m_get_mono(rda5807m_t *dev, bool *mono)
{
    CHECK_ARG(dev);
    CHECK_ARG(mono);

    uint16_t v;
    CHECK(read_register(dev, REG_CTRL, &v));
    *mono = v & BV(BIT_CTRL_MONO);

    return ESP_OK;
}

esp_err_t rda5807m_set_mono(rda5807m_t *dev, bool mono)
{
    CHECK_ARG(dev);

    CHECK(update_register(dev, REG_CTRL, BV(BIT_CTRL_MONO), mono ? BV(BIT_CTRL_MONO) : 0));

    ESP_LOGI(TAG, "Mono %s", mono ? "enabled" : "disabled");

    return ESP_OK;
}

esp_err_t rda5807m_get_band(rda5807m_t *dev, rda5807m_band_t *band)
{
    CHECK_ARG(dev);
    CHECK_ARG(band);

    uint16_t v;
    CHECK(read_register(dev, REG_CHAN, &v));
    v = (v & MASK_CHAN_BAND) >> BIT_CHAN_BAND;
    if (v == 3)
    {
        uint16_t r7;
        CHECK(read_register(dev, REG_R7, &r7));
        *band = (r7 >> BIT_R7_50M) & 1 ? RDA5807M_BAND_65_76 : RDA5807M_BAND_50_76;
    }
    else *band = v;

    dev->band = *band;

    return ESP_OK;
}

esp_err_t rda5807m_set_band(rda5807m_t *dev, rda5807m_band_t band)
{
    CHECK_ARG(dev);
    CHECK_ARG(band <= RDA5807M_BAND_50_76);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, update_register_nolock(dev, REG_CHAN, MASK_CHAN_BAND,
            (band <= RDA5807M_BAND_65_76 ? band : RDA5807M_BAND_65_76) << BIT_CHAN_BAND));
    if (band >= RDA5807M_BAND_65_76)
        I2C_DEV_CHECK(&dev->i2c_dev, update_register_nolock(dev, REG_R7, BV(BIT_R7_50M),
                band == RDA5807M_BAND_65_76 ? BV(BIT_R7_50M) : 0));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->band = band;

    ESP_LOGI(TAG, "Band: %d..%d kHz", band_limits[band].lower, band_limits[band].upper);

    return ESP_OK;
}

esp_err_t rda5807m_get_channel_spacing(rda5807m_t *dev, rda5807m_channel_spacing_t *spacing)
{
    CHECK_ARG(dev);
    CHECK_ARG(spacing);

    uint16_t v;
    CHECK(read_register(dev, REG_CHAN, &v));
    dev->spacing = *spacing = v & MASK_CHAN_SPACE;

    return ESP_OK;
}

esp_err_t rda5807m_set_channel_spacing(rda5807m_t *dev, rda5807m_channel_spacing_t spacing)
{
    CHECK_ARG(dev);
    CHECK_ARG(spacing <= RDA5807M_CHAN_SPACE_25);

    CHECK(update_register(dev, REG_CTRL, MASK_CHAN_SPACE, spacing));
    dev->spacing = spacing;

    return ESP_OK;
}

esp_err_t rda5807m_get_frequency_khz(rda5807m_t *dev, uint32_t *frequency)
{
    CHECK_ARG(dev);
    CHECK_ARG(frequency);

    uint16_t chan;
    CHECK(read_registers_bulk(dev, &chan, 1));
    chan &= MASK_RA_READCHAN;

    *frequency = chan * spacings[dev->spacing] + band_limits[dev->band].lower;

    return ESP_OK;
}

esp_err_t rda5807m_set_frequency_khz(rda5807m_t *dev, uint32_t frequency)
{
    CHECK_ARG(dev);

    if (frequency < band_limits[dev->band].lower || frequency > band_limits[dev->band].upper)
    {
        ESP_LOGE(TAG, "Could not set frequency: %d kHz is out of bounds (%d..%d)",
                frequency, band_limits[dev->band].lower, band_limits[dev->band].upper);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t chan = (frequency - band_limits[dev->band].lower) / spacings[dev->spacing];
    if (chan > MAX_CHAN)
    {
        ESP_LOGE(TAG, "Could not set frequency to %d kHz with current band/spacing settings", frequency);
        return ESP_ERR_INVALID_ARG;
    }

    CHECK(update_register(dev, REG_CHAN, MASK_CHAN_CHAN | BV(BIT_CHAN_TUNE),
            (chan << BIT_CHAN_CHAN) | BV(BIT_CHAN_TUNE)));

    ESP_LOGI(TAG, "Frequency: %d kHz", chan * spacings[dev->spacing] + band_limits[dev->band].lower);

    return ESP_OK;
}

esp_err_t rda5807m_get_afc(rda5807m_t *dev, bool *afc)
{
    CHECK_ARG(dev);
    CHECK_ARG(afc);

    uint16_t v;
    CHECK(read_register(dev, REG_R4, &v));
    *afc = !(v & BV(BIT_R4_AFCD));

    return ESP_OK;
}

esp_err_t rda5807m_set_afc(rda5807m_t *dev, bool afc)
{
    CHECK_ARG(dev);

    return update_register(dev, REG_R4, BV(BIT_R4_AFCD), afc ? 0 : BV(BIT_R4_AFCD));
}

esp_err_t rda5807m_seek_start(rda5807m_t *dev, bool up, bool wrap, uint8_t threshold)
{
    CHECK_ARG(dev);
    CHECK_ARG(threshold <= RDA5807M_SEEK_TH_MAX);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, update_register_nolock(dev, REG_VOL, MASK_VOL_SEEKTH,
            threshold << BIT_VOL_SEEKTH));
    I2C_DEV_CHECK(&dev->i2c_dev, update_register_nolock(dev, REG_CTRL,
            BV(BIT_CTRL_SEEK) | BV(BIT_CTRL_SEEKUP) | BV(BIT_CTRL_SKMODE),
            BV(BIT_CTRL_SEEK) | (up ? BV(BIT_CTRL_SEEKUP) : 0) | (wrap ? 0 : BV(BIT_CTRL_SKMODE))));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGI(TAG, "Seek started: %s, %s at bound, SNR threshold: %d", up ? "up" : "down", wrap ? "wrap" : "stop", threshold);

    return ESP_OK;
}

esp_err_t rda5807m_seek_stop(rda5807m_t *dev)
{
    CHECK_ARG(dev);
    CHECK(update_register(dev, REG_CTRL, BV(BIT_CTRL_SEEK), 0));

    ESP_LOGI(TAG, "Seek stopped");

    return ESP_OK;
}


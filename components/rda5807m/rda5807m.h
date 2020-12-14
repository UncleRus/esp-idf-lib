/**
 * @file rda5807m.h
 * @defgroup rda5807m rda5807m
 * @{
 *
 * ESP-IDF driver for single-chip broadcast FM radio tuner RDA5807M
 *
 * Copyright (C) 2018 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __RDA5807M_H__
#define __RDA5807M_H__

#include <i2cdev.h>
#include <stdbool.h>

#define RDA5807M_RSSI_MAX    0x3f
#define RDA5807M_SEEK_TH_DEF 0x08
#define RDA5807M_SEEK_TH_MAX 0x0f
#define RDA5807M_VOL_MAX     0x0f

/**
 * Clock mode
 */
typedef enum
{
    RDA5807M_CLK_32768HZ = 0, //!< 32768 Hz, default
    RDA5807M_CLK_12MHZ   = 1, //!< 12 MHz
    RDA5807M_CLK_13MHZ   = 2, //!< 13 MHz
    RDA5807M_CLK_19_2MHZ = 3, //!< 19.2 MHz
    RDA5807M_CLK_24MHZ   = 5, //!< 24 MHz
    RDA5807M_CLK_26MHZ   = 6, //!< 26 MHz
    RDA5807M_CLK_38_4MHZ = 7, //!< 38.4 MHz
} rda5807m_clock_freq_t;

/**
 * Channel spacing (frequency step)
 */
typedef enum
{
    RDA5807M_CHAN_SPACE_100 = 0, //!< 100 KHz, default
    RDA5807M_CHAN_SPACE_200,     //!< 200 KHz
    RDA5807M_CHAN_SPACE_50,      //!< 50 KHz
    RDA5807M_CHAN_SPACE_25       //!< 25 KHz
} rda5807m_channel_spacing_t;

/**
 * FM Band
 */
typedef enum {
    RDA5807M_BAND_87_108 = 0, //!< 87..108 MHz (US/Europe), default
    RDA5807M_BAND_76_91,      //!< 76..91 MHz (Japan)
    RDA5807M_BAND_76_108,     //!< 76..108 MHz (Worldwide)
    RDA5807M_BAND_65_76,      //!< 65..76 MHz (Eastern Europe)
    RDA5807M_BAND_50_76       //!< 50..76 MHz (Eastern Europe wide)
} rda5807m_band_t;

/**
 * Seek status
 */
typedef enum {
    RDA5807M_SEEK_NONE = 0, //!< No seeking stations currently
    RDA5807M_SEEK_STARTED,  //!< Seeking in progress
    RDA5807M_SEEK_COMPLETE, //!< Seeking complete
    RDA5807M_SEEK_FAILED    //!< Seeking failed - no stations with RSSI > threshold found
} rda5807m_seek_status_t;

/**
 * Overall device status
 */
typedef struct
{
    rda5807m_seek_status_t seek_status; //!< Seek status
    bool station;                       //!< True if tuned to a station
    bool stereo;                        //!< True if stereo is available
    bool rds_ready;                     //!< True if RDS data is ready
    uint8_t rssi;                       //!< RSSI, 0..RDA5807M_RSSI_MAX (logarithmic scale)
    uint32_t frequency;                 //!< Current frequency, kHz
    uint16_t rds[4];                    //!< RDS data
} rda5807m_state_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;                  //!< I2C device descriptor
    rda5807m_band_t band;               //!< Current band
    rda5807m_channel_spacing_t spacing; //!< Current spacing
} rda5807m_t;

/**
 * Initialize device descriptor
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_init_desc(rda5807m_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * Free device descriptor
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_free_desc(rda5807m_t *dev);

/**
 * Initialize device
 * @param dev Device descriptor
 * @param clock_freq RCLK frequency, usually `RDA5807M_CLK_32768HZ`
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_init(rda5807m_t *dev, rda5807m_clock_freq_t clock_freq);

/**
 * Get current device status
 * @param dev Device descriptor
 * @param state Device status descriptor
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_state(rda5807m_t *dev, rda5807m_state_t *state);

/**
 * Get volume level (DAC gain)
 * @param dev Device descriptor
 * @param vol Volume level, 0..RDA5807M_VOL_MAX
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_volume(rda5807m_t *dev, uint8_t *vol);

/**
 * Set volume level (DAC gain)
 * Volume scale is logarithmic. When 0, device is muted and output impedance
 * is very large.
 * @param dev Device descriptor
 * @param vol Volume level, 0..RDA5807M_VOL_MAX
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_volume(rda5807m_t *dev, uint8_t vol);

/**
 * Get current mute state
 * @param dev Device descriptor
 * @param mute Mute state
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_mute(rda5807m_t *dev, bool *mute);

/**
 * Mute/unmute device
 * @param dev Device descriptor
 * @param mute Mute if true
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_mute(rda5807m_t *dev, bool mute);

/**
 * Get current soft mute state
 * @param dev Device descriptor
 * @param softmute Soft mute state
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_softmute(rda5807m_t *dev, bool *softmute);

/**
 * Enable/disable soft mute
 * @param dev Device descriptor
 * @param softmute If true, enable soft mute
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_softmute(rda5807m_t *dev, bool softmute);

/**
 * Get current state of the bass boost feature
 * @param dev Device descriptor
 * @param bass_boost Bass boost state
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_bass_boost(rda5807m_t *dev, bool *bass_boost);

/**
 * Enable/disable bass boost feature
 * @param dev Device descriptor
 * @param bass_boost If true, enable bass boost
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_bass_boost(rda5807m_t *dev, bool bass_boost);

/**
 * Get forced mono state
 * @param dev Device descriptor
 * @param mono Forced mono state
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_mono(rda5807m_t *dev, bool *mono);

/**
 * Enable/disable forced mono
 * @param dev Device descriptor
 * @param mono If true, audio will be in mono even if stereo is available
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_mono(rda5807m_t *dev, bool mono);

/**
 * Get current band
 * @param dev Device descriptor
 * @param band Current band
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_band(rda5807m_t *dev, rda5807m_band_t *band);

/**
 * Switch device to band
 * @param dev Device descriptor
 * @param band New band
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_band(rda5807m_t *dev, rda5807m_band_t band);

/**
 * Get current channel spacing (frequency step)
 * @param dev Device descriptor
 * @param spacing Current channel spacing
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_channel_spacing(rda5807m_t *dev, rda5807m_channel_spacing_t *spacing);

/**
 * Set channel spacing (frequency step)
 * @param dev Device descriptor
 * @param spacing Channel spacing, usually `RDA5807M_CHAN_SPACE_100`
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_channel_spacing(rda5807m_t *dev, rda5807m_channel_spacing_t spacing);

/**
 * Get frequency the device is tuned to
 * @param dev Device descriptor
 * @param frequency Frequency, kHz
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_frequency_khz(rda5807m_t *dev, uint32_t *frequency);

/**
 * Tune device to a frequency
 * @param dev Device descriptor
 * @param frequency Frequency, kHz
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_frequency_khz(rda5807m_t *dev, uint32_t frequency);

/**
 * Get current state of the automatic frequency control
 * @param dev Device descriptor
 * @param afc AFC state
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_get_afc(rda5807m_t *dev, bool *afc);

/**
 * Enable/disable automatic frequency control
 * @param dev Device descriptor
 * @param afc AFC state
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_set_afc(rda5807m_t *dev, bool afc);

/**
 * Start seeking stations
 * @param dev Device descriptor
 * @param up Seeking direction: true - up, false - down
 * @param wrap If true, wrap at the upper or lower band limit and
 *             continue seeking, else stop seeking at bounds
 * @param threshold Seeking SNR treshold, 0..`RDA5807M_SEEK_TH_MAX`.
 *                  Usually it's `RDA5807M_SEEK_TH_DEF`
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_seek_start(rda5807m_t *dev, bool up, bool wrap, uint8_t threshold);

/**
 * Stop seeking stations
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t rda5807m_seek_stop(rda5807m_t *dev);

/**@}*/

#endif /* __RDA5807M_H__ */

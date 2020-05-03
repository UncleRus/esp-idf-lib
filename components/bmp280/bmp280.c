/**
 * @file bmp280.c
 *
 * ESP-IDF driver for BMP280/BME280 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016 sheinz <https://github.com/sheinz>\n
 * Copyright (C) 2018 Ruslan V. Uss <https://github.com/UncleRus>
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "bmp280.h"

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf

static const char *TAG = "BMP280";

/**
 * BMP280 registers
 */
#define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_HUM_CALIB   0x88

#define BMP280_RESET_VALUE     0xB6

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(dev, x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

static esp_err_t read_register16(i2c_dev_t *dev, uint8_t reg, uint16_t *r)
{
    uint8_t d[] = { 0, 0 };

    CHECK(i2c_dev_read_reg(dev, reg, d, 2));
    *r = d[0] | (d[1] << 8);

    return ESP_OK;
}

inline static esp_err_t write_register8(i2c_dev_t *dev, uint8_t addr, uint8_t value)
{
    return i2c_dev_write_reg(dev, addr, &value, 1);
}

static esp_err_t read_calibration_data(bmp280_t *dev)
{
    CHECK(read_register16(&dev->i2c_dev, 0x88, &dev->dig_T1));
    CHECK(read_register16(&dev->i2c_dev, 0x8a, (uint16_t *)&dev->dig_T2));
    CHECK(read_register16(&dev->i2c_dev, 0x8c, (uint16_t *)&dev->dig_T3));
    CHECK(read_register16(&dev->i2c_dev, 0x8e, &dev->dig_P1));
    CHECK(read_register16(&dev->i2c_dev, 0x90, (uint16_t *)&dev->dig_P2));
    CHECK(read_register16(&dev->i2c_dev, 0x92, (uint16_t *)&dev->dig_P3));
    CHECK(read_register16(&dev->i2c_dev, 0x94, (uint16_t *)&dev->dig_P4));
    CHECK(read_register16(&dev->i2c_dev, 0x96, (uint16_t *)&dev->dig_P5));
    CHECK(read_register16(&dev->i2c_dev, 0x98, (uint16_t *)&dev->dig_P6));
    CHECK(read_register16(&dev->i2c_dev, 0x9a, (uint16_t *)&dev->dig_P7));
    CHECK(read_register16(&dev->i2c_dev, 0x9c, (uint16_t *)&dev->dig_P8));
    CHECK(read_register16(&dev->i2c_dev, 0x9e, (uint16_t *)&dev->dig_P9));

    ESP_LOGD(TAG, "Calibration data received:");
    ESP_LOGD(TAG, "dig_T1=%d", dev->dig_T1);
    ESP_LOGD(TAG, "dig_T2=%d", dev->dig_T2);
    ESP_LOGD(TAG, "dig_T3=%d", dev->dig_T3);
    ESP_LOGD(TAG, "dig_P1=%d", dev->dig_P1);
    ESP_LOGD(TAG, "dig_P2=%d", dev->dig_P2);
    ESP_LOGD(TAG, "dig_P3=%d", dev->dig_P3);
    ESP_LOGD(TAG, "dig_P4=%d", dev->dig_P4);
    ESP_LOGD(TAG, "dig_P5=%d", dev->dig_P5);
    ESP_LOGD(TAG, "dig_P6=%d", dev->dig_P6);
    ESP_LOGD(TAG, "dig_P7=%d", dev->dig_P7);
    ESP_LOGD(TAG, "dig_P8=%d", dev->dig_P8);
    ESP_LOGD(TAG, "dig_P9=%d", dev->dig_P9);

    return ESP_OK;
}

static esp_err_t read_hum_calibration_data(bmp280_t *dev)
{
    uint16_t h4, h5;

    CHECK(i2c_dev_read_reg(&dev->i2c_dev, 0xa1, &dev->dig_H1, 1));
    CHECK(read_register16(&dev->i2c_dev, 0xe1, (uint16_t *)&dev->dig_H2));
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, 0xe3, &dev->dig_H3, 1));
    CHECK(read_register16(&dev->i2c_dev, 0xe4, &h4));
    CHECK(read_register16(&dev->i2c_dev, 0xe5, &h5));
    CHECK(i2c_dev_read_reg(&dev->i2c_dev, 0xe7, (uint8_t *)&dev->dig_H6, 1));

    dev->dig_H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
    dev->dig_H5 = h5 >> 4;
    ESP_LOGD(TAG, "Calibration data received:");
    ESP_LOGD(TAG, "dig_H1=%d", dev->dig_H1);
    ESP_LOGD(TAG, "dig_H2=%d", dev->dig_H2);
    ESP_LOGD(TAG, "dig_H3=%d", dev->dig_H3);
    ESP_LOGD(TAG, "dig_H4=%d", dev->dig_H4);
    ESP_LOGD(TAG, "dig_H5=%d", dev->dig_H5);
    ESP_LOGD(TAG, "dig_H6=%d", dev->dig_H6);

    return ESP_OK;
}

esp_err_t bmp280_init_desc(bmp280_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != BMP280_I2C_ADDRESS_0 && addr != BMP280_I2C_ADDRESS_1)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t bmp280_free_desc(bmp280_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bmp280_init_default_params(bmp280_params_t *params)
{
    CHECK_ARG(params);

    params->mode = BMP280_MODE_NORMAL;
    params->filter = BMP280_FILTER_OFF;
    params->oversampling_pressure = BMP280_STANDARD;
    params->oversampling_temperature = BMP280_STANDARD;
    params->oversampling_humidity = BMP280_STANDARD;
    params->standby = BMP280_STANDBY_250;

    return ESP_OK;
}

esp_err_t bmp280_init(bmp280_t *dev, bmp280_params_t *params)
{
    CHECK_ARG(dev && params);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    CHECK_LOGE(dev, i2c_dev_read_reg(&dev->i2c_dev, BMP280_REG_ID, &dev->id, 1), "Sensor not found");

    if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID)
    {
        CHECK_LOGE(dev, ESP_ERR_INVALID_VERSION,
                "Invalid chip ID: expected: 0x%x (BME280) or 0x%x (BMP280) got: 0x%x",
                BME280_CHIP_ID, BMP280_CHIP_ID, dev->id);
    }

    // Soft reset.
    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_RESET, BMP280_RESET_VALUE), "Failed to reset sensor");

    // Wait until finished copying over the NVP data.
    while (1)
    {
        uint8_t status;
        if (!i2c_dev_read_reg(&dev->i2c_dev, BMP280_REG_STATUS, &status, 1) && (status & 1) == 0)
            break;
    }

    CHECK_LOGE(dev, read_calibration_data(dev), "Failed to read calibration data");

    if (dev->id == BME280_CHIP_ID)
    {
        CHECK_LOGE(dev, read_hum_calibration_data(dev), "Failed to read humidity calibration data");
    }

    uint8_t config = (params->standby << 5) | (params->filter << 2);
    ESP_LOGD(TAG, "Writing config reg=%x", config);

    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_CONFIG, config), "Failed to configure sensor");

    if (params->mode == BMP280_MODE_FORCED)
    {
        params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
    }

    uint8_t ctrl = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2) | (params->mode);

    if (dev->id == BME280_CHIP_ID)
    {
        // Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
        uint8_t ctrl_hum = params->oversampling_humidity;
        ESP_LOGD(TAG, "Writing ctrl hum reg=%x", ctrl_hum);
        CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_CTRL_HUM, ctrl_hum), "Failed to control sensor");
    }

    ESP_LOGD(TAG, "Writing ctrl reg=%x", ctrl);
    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_CTRL, ctrl), "Failed to control sensor");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmp280_force_measurement(bmp280_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t ctrl;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BMP280_REG_CTRL, &ctrl, 1));
    ctrl &= ~0b11;  // clear two lower bits
    ctrl |= BMP280_MODE_FORCED;
    ESP_LOGD(TAG, "Writing ctrl reg=%x", ctrl);
    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, BMP280_REG_CTRL, ctrl), "Failed to start forced mode");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmp280_is_measuring(bmp280_t *dev, bool *busy)
{
    CHECK_ARG(dev && busy);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    const uint8_t regs[2] = { BMP280_REG_STATUS, BMP280_REG_CTRL };
    uint8_t status[2];
    CHECK_LOGE(dev, i2c_dev_read(&dev->i2c_dev, regs, 2, status, 2), "Failed to read status registers");

    // Check mode - FORCED means BM280 is busy (it switches to SLEEP mode when finished)
    // Additionally, check 'measuring' bit in status register
    *busy = ((status[1] & 0b11) == BMP280_MODE_FORCED) || (status[0] & (1 << 3));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
static inline int32_t compensate_temperature(bmp280_t *dev, int32_t adc_temp, int32_t *fine_temp)
{
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t)dev->dig_T1 << 1))) * (int32_t)dev->dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t)dev->dig_T1) * ((adc_temp >> 4) - (int32_t)dev->dig_T1)) >> 12) * (int32_t)dev->dig_T3) >> 14;

    *fine_temp = var1 + var2;
    return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(bmp280_t *dev, int32_t adc_press, int32_t fine_temp)
{
    int64_t var1, var2, p;

    var1 = (int64_t)fine_temp - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dev->dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;  // avoid exception caused by division by zero
    }

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dev->dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dev->dig_P7 << 4);
    return p;
}

/**
 * Compensation algorithm is taken from BME280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_humidity(bmp280_t *dev, int32_t adc_hum, int32_t fine_temp)
{
    int32_t v_x1_u32r;

    v_x1_u32r = fine_temp - (int32_t)76800;
    v_x1_u32r = ((((adc_hum << 14) - ((int32_t)dev->dig_H4 << 20) - ((int32_t)dev->dig_H5 * v_x1_u32r)) + (int32_t)16384) >> 15)
            * (((((((v_x1_u32r * (int32_t)dev->dig_H6) >> 10) * (((v_x1_u32r * (int32_t)dev->dig_H3) >> 11) + (int32_t)32768)) >> 10)
                    + (int32_t)2097152) * (int32_t)dev->dig_H2 + 8192) >> 14);
    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (int32_t)dev->dig_H1) >> 4);
    v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
    v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
    return v_x1_u32r >> 12;
}

esp_err_t bmp280_read_fixed(bmp280_t *dev, int32_t *temperature, uint32_t *pressure, uint32_t *humidity)
{
    CHECK_ARG(dev && temperature && pressure);

    int32_t adc_pressure;
    int32_t adc_temp;
    uint8_t data[8];

    // Only the BME280 supports reading the humidity.
    if (dev->id != BME280_CHIP_ID)
    {
        if (humidity)
            *humidity = 0;
        humidity = NULL;
    }

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // Need to read in one sequence to ensure they match.
    size_t size = humidity ? 8 : 6;
    CHECK_LOGE(dev, i2c_dev_read_reg(&dev->i2c_dev, 0xf7, data, size), "Failed to read data");

    adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;
    ESP_LOGD(TAG, "ADC temperature: %d", adc_temp);
    ESP_LOGD(TAG, "ADC pressure: %d", adc_pressure);

    int32_t fine_temp;
    *temperature = compensate_temperature(dev, adc_temp, &fine_temp);
    *pressure = compensate_pressure(dev, adc_pressure, fine_temp);

    if (humidity)
    {
        int32_t adc_humidity = data[6] << 8 | data[7];
        ESP_LOGD(TAG, "ADC humidity: %d", adc_humidity);
        *humidity = compensate_humidity(dev, adc_humidity, fine_temp);
    }

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmp280_read_float(bmp280_t *dev, float *temperature, float *pressure, float *humidity)
{
    int32_t fixed_temperature;
    uint32_t fixed_pressure;
    uint32_t fixed_humidity;
    CHECK(bmp280_read_fixed(dev, &fixed_temperature, &fixed_pressure, humidity ? &fixed_humidity : NULL));
    *temperature = (float)fixed_temperature / 100;
    *pressure = (float)fixed_pressure / 256;
    if (humidity)
        *humidity = (float)fixed_humidity / 1024;

    return ESP_OK;
}

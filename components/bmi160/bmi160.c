/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2025 Lukasz Bielinski <lbielinski01@gmail.com>
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

 #include <freertos/FreeRTOS.h>
 #include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <esp_err.h>
#include <string.h>

#include "bmi160.h"

static const char *TAG = "BMI160";

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 2.44Mhz

#define I2C_PORT 0
#define WARNING_CHANNEL 1
#define WARNING_CURRENT (40.0)

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 2.44Mhz

static bool is_acc_odr_fits_mode(bmi160_acc_odr_t odr, bmi160_pmu_acc_mode_t mode, bmi160_acc_lp_avg_t avg);
static bool is_gyr_mode_valid(bmi160_pmu_gyr_mode_t mode);
static bool is_acc_mode_valid(bmi160_pmu_acc_mode_t mode);


esp_err_t bmi160_read_reg(bmi160_t *dev, uint8_t reg, uint8_t *val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, val, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_read_reg_array(bmi160_t *dev, uint8_t reg, uint8_t *val, uint8_t num)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, val, num));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_write_reg(bmi160_t *dev, uint8_t reg, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_write_reg_array(bmi160_t *dev, uint8_t reg, uint8_t* val, uint8_t num)
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg, val, num));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_init(bmi160_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    // esp_err_t ret = i2cdev_init();
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to init I2C");
    //     return ret;
    // }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    memset(&dev->aBias, 0, sizeof(dev->aBias));
    memset(&dev->gBias, 0, sizeof(dev->gBias));


    
    if (i2c_dev_create_mutex(&dev->i2c_dev) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bmi160_free(bmi160_t *dev)
{
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bmi160_set_acc_range(bmi160_t *dev, bmi160_acc_range_t range)
{
    dev->accRange = range;

    switch(range){
        case BMI160_ACC_RANGE_2G:
            dev->aRes = 2.0f / 32768.0f;
            break;
        case BMI160_ACC_RANGE_4G:
            dev->aRes = 4.0f / 32768.0f;
            break;
        case BMI160_ACC_RANGE_8G:
            dev->aRes = 8.0f / 32768.0f;
            break;
        case BMI160_ACC_RANGE_16G:
            dev->aRes = 16.0f / 32768.0f;
            break;
        default:
            return ESP_FAIL;
    }

	return bmi160_write_reg(dev, BMI160_ACC_RANGE, range);  // Set up scale Accel range.
}

esp_err_t bmi160_set_gyr_range(bmi160_t *dev, bmi160_gyr_range_t range)
{
    dev->gyrRange = range;

    switch(range){
        case BMI160_GYR_RANGE_2000DPS:
            dev->gRes = 2000.0f / 32768.0f;
            break;
        case BMI160_GYR_RANGE_1000DPS:
            dev->gRes = 1000.0f / 32768.0f;
            break;
        case BMI160_GYR_RANGE_500DPS:
            dev->gRes = 500.0f / 32768.0f;
            break;
        case BMI160_GYR_RANGE_250DPS:
            dev->gRes = 250.0f / 32768.0f;
            break;
        case BMI160_GYR_RANGE_125DPS:
            dev->gRes = 125.0f / 32768.0f;
            break;
        default:
            return ESP_FAIL;
    }

    return bmi160_write_reg(dev, BMI160_GYR_RANGE, range);  // Set up scale Gyro range.
}

esp_err_t bmi160_set_acc_conf(bmi160_t *dev, bmi160_acc_odr_t odr, bmi160_acc_lp_avg_t avg, bmi160_acc_us_t acc_us)
{  
    dev->accOdr = odr;
    dev->accConf = (odr & 0x0Fu) | ((avg & 0x07u) << 4) | ((acc_us & 0x1u) << 7);

    return bmi160_write_reg(dev, BMI160_ACC_CONF,  dev->accConf);  // Set Accel CONF
}

esp_err_t bmi160_set_gyr_odr(bmi160_t *dev, bmi160_gyr_odr_t odr)
{
    dev->gyrOdr = odr;

    return bmi160_write_reg(dev, BMI160_GYR_CONF, odr);  // Set Gyro ODR
}

esp_err_t bmi160_read_data(bmi160_t *dev, bmi160_result_t *result)
{
    uint8_t rawData[12];
    bmi160_read_reg_array(dev, BMI160_GYR_X_L, rawData, 12);

    int16_t data[6];
    //loop to convert 2 8bit values to 16 bit value
    for(int i = 0; i < 6; i++)
    {
        data[i] = ((int16_t)(rawData[i*2+1]) << 8) | rawData[i*2];
    }

    result->accX = ((float)data[3] * dev->aRes) - dev->aBias[0]; //acceleration x
    result->accY = ((float)data[4] * dev->aRes) - dev->aBias[1]; //acceleration y
    result->accZ = ((float)data[5] * dev->aRes) - dev->aBias[2]; //acceleration z
    result->gyroX = ((float)data[0] * dev->gRes) - dev->gBias[0]; //gyro x
    result->gyroY = ((float)data[1] * dev->gRes) - dev->gBias[1]; //gyro y
    result->gyroZ = ((float)data[2] * dev->gRes) - dev->gBias[2]; //gyro z

    return ESP_OK;
}

static bool is_acc_mode_valid(bmi160_pmu_acc_mode_t mode)
{
    switch(mode)
    {
        case BMI160_PMU_ACC_SUSPEND:
        case BMI160_PMU_ACC_NORMAL:
        case BMI160_PMU_ACC_LOW_POWER:
            return true;
        default:
            return false;
    }
}

static bool is_gyr_mode_valid(bmi160_pmu_gyr_mode_t mode)
{
    switch(mode)
    {
        case BMI160_PMU_GYR_SUSPEND:
        case BMI160_PMU_GYR_NORMAL:
        case BMI160_PMU_GYR_FAST_STARTUP:
            return true;
        default:
            return false;
    }
}

static bool is_acc_odr_fits_mode(bmi160_acc_odr_t odr, bmi160_pmu_acc_mode_t mode, bmi160_acc_lp_avg_t avg)
{
    bool result = false;
    switch(mode)
    {
        case BMI160_PMU_ACC_SUSPEND:
            result = true;
            break;
        case BMI160_PMU_ACC_NORMAL:
            result = (odr >= BMI160_ACC_ODR_12_5HZ);
            break;
        case BMI160_PMU_ACC_LOW_POWER:
            if((odr == BMI160_ACC_ODR_400HZ) && (avg <= BMI160_ACC_LP_AVG_2))
            {
                result = true;
            }
            else if((odr == BMI160_ACC_ODR_200HZ) && (avg <= BMI160_ACC_LP_AVG_4))
            {
                result = true;
            }
            else if((odr == BMI160_ACC_ODR_100HZ) && (avg <= BMI160_ACC_LP_AVG_8))
            {
                result = true;
            }
            else if((odr == BMI160_ACC_ODR_50HZ) && (avg <= BMI160_ACC_LP_AVG_16))
            {
                result = true;
            }
            else if((odr == BMI160_ACC_ODR_25HZ) && (avg <= BMI160_ACC_LP_AVG_32))
            {
                result = true;
            }
            else if((odr == BMI160_ACC_ODR_12_5HZ) && (avg <= BMI160_ACC_LP_AVG_64))
            {
                result = true;
            }
            else if((odr <= BMI160_ACC_ODR_6_25HZ) && (avg <= BMI160_ACC_LP_AVG_128))
            {
                result = true;
            }
            else
            {
                result = false;
            }
            break;
        default:
            result = false;
            break;
    }

    return result;
}

esp_err_t bmi160_start(bmi160_t *dev, bmi160_conf_t* conf)
{
    //read device id
    uint8_t device_id;
    bmi160_read_reg(dev, BMI160_CHIP_ID, &device_id);
    ESP_LOGD(TAG,"Device ID: 0x%02x\n", device_id);
    if(device_id != BMI160_CHIP_ID_DEFAULT_VALUE)
    {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02x", device_id);
        return ESP_FAIL;
    }

    //read error status
    uint8_t err;
    bmi160_read_reg(dev, BMI160_ERR_REG, &err);
    ESP_LOGD(TAG,"Error: 0x%02x\n", err);
    if(err != 0)
    {
        ESP_LOGE(TAG, "Error: 0x%02x", err);
    }

    // //pmu status
    // uint8_t pmu_status;
    // bmi160_read_reg(dev, BMI160_PMU_STATUS, &pmu_status);
    // printf("ACC PMU Status: 0x%02x\n", (pmu_status & 0x30) >> 4);
    // printf("GYR PMU Status: 0x%02x\n", (pmu_status & 0x0C) >> 2);
    // printf("MAG PMU Status: 0x%02x\n", (pmu_status & 0x03));

    dev->accRange = conf->accRange;
    dev->gyrRange = conf->gyrRange;
    dev->accOdr = conf->accOdr;
    dev->gyrOdr = conf->gyrOdr;

    //validate parameters
    if(!is_acc_mode_valid(conf->accMode))
    {
        ESP_LOGE(TAG, "Invalid Accelerometer Mode");
        return ESP_FAIL;
    }
    if(!is_gyr_mode_valid(conf->gyrMode))
    {
        ESP_LOGE(TAG, "Invalid Gyroscope Mode");
        return ESP_FAIL;
    }
    if(!is_acc_odr_fits_mode(conf->accOdr, conf->accMode, conf->accAvg))
    {
        ESP_LOGE(TAG, "Invalid Accelerometer ODR for the mode");
        return ESP_FAIL;
    }

    //reset device
    bmi160_write_reg(dev, BMI160_CMD, 0xB6); // toggle software reset
    //delay 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t pmu_status;
    if(conf->accMode != BMI160_PMU_ACC_SUSPEND)
    {
        //start up accelerometer
        bmi160_write_reg(dev, BMI160_CMD, conf->accMode);
        vTaskDelay(pdMS_TO_TICKS(100));
        bmi160_read_reg(dev, BMI160_PMU_STATUS, &pmu_status);
        if(((pmu_status & 0x30) >> 4) != (conf->accMode & 0x3))
        {
            ESP_LOGE(TAG, "Accelerometer PMU status: 0x%02x", (pmu_status & 0x30) >> 4);
            return ESP_FAIL;
        }
        if(ESP_FAIL == bmi160_set_acc_range(dev, conf->accRange))
        {
            ESP_LOGE(TAG, "Invalid Accelerometer Range");
            return ESP_FAIL;
        }
        if(ESP_FAIL == bmi160_set_acc_conf(dev, conf->accOdr, conf->accAvg, conf->accUs))
        {
            ESP_LOGE(TAG, "Invalid Accelerometer configuration");
            return ESP_FAIL;
        }
    }

    if(conf->gyrMode != BMI160_PMU_GYR_SUSPEND)
    {
        //start up gyroscope
        bmi160_write_reg(dev, BMI160_CMD, conf->gyrMode);
        vTaskDelay(pdMS_TO_TICKS(100));
        bmi160_read_reg(dev, BMI160_PMU_STATUS, &pmu_status);
        if(((pmu_status & 0x0C) >> 2) != (conf->gyrMode & 0x3))
        {
            ESP_LOGE(TAG, "Gyroscope PMU status: 0x%02x", (pmu_status & 0x0C) >> 2);
            return ESP_FAIL;
        }
        if(ESP_FAIL == bmi160_set_gyr_range(dev, conf->gyrRange))
        {
            ESP_LOGE(TAG, "Invalid Gyroscope Range");
            return ESP_FAIL;
        }
        if(ESP_FAIL == bmi160_set_gyr_odr(dev, conf->gyrOdr))
        {
            ESP_LOGE(TAG, "Invalid Gyroscope ODR");
            return ESP_FAIL;
        }
    }
    
    //pmu status
    bmi160_read_reg(dev, BMI160_PMU_STATUS, &pmu_status);
    ESP_LOGD(TAG,"ACC PMU Status: 0x%02x\n", (pmu_status & 0x30) >> 4);
    ESP_LOGD(TAG,"GYR PMU Status: 0x%02x\n", (pmu_status & 0x0C) >> 2);
    ESP_LOGD(TAG,"MAG PMU Status: 0x%02x\n", (pmu_status & 0x03));

    return ESP_OK;
}

esp_err_t bmi160_calibrate(bmi160_t *dev)
{
    //calibrate accelerometer and gyroscope to calculate bias from 64 readings in 20 ms period

    bmi160_result_t result;
    float accX = 0, accY = 0, accZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0;
    for(int i = 0; i < 64; i++)
    {
        bmi160_read_data(dev, &result);
        accX += result.accX;
        accY += result.accY;
        accZ += result.accZ;
        gyroX += result.gyroX;
        gyroY += result.gyroY;
        gyroZ += result.gyroZ;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    //calculate average
    accX /= 64.0f;
    accY /= 64.0f;
    accZ /= 64.0f;
    gyroX /= 64.0f;
    gyroY /= 64.0f;
    gyroZ /= 64.0f;

    //store bias values
    dev->aBias[0] = accX;
    dev->aBias[1] = accY;
    dev->aBias[2] = accZ;
    dev->gBias[0] = gyroX;
    dev->gBias[1] = gyroY;
    dev->gBias[2] = gyroZ;

    //print bias values
    ESP_LOGD(TAG,"Accel Bias: %+.3f %+.3f %+.3f Gyro Bias: %+.3f %+.3f %+.3f\n", dev->aBias[0], dev->aBias[1], dev->aBias[2], dev->gBias[0], dev->gBias[1], dev->gBias[2]);

    return ESP_OK;
}

/**
 * @brief self test for the BMI160
 * 
 * @note 
 * 
 * @param dev 
 * @return esp_err_t 
 */
esp_err_t bmi160_self_test(bmi160_t *dev)
{
    //read device id
    uint8_t device_id;
    bmi160_read_reg(dev, BMI160_CHIP_ID, &device_id);
    printf("Device ID: 0x%02x\n", device_id);
    if(device_id != BMI160_CHIP_ID_DEFAULT_VALUE)
    {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02x", device_id);
        return ESP_FAIL;
    }
    
    //reset device
    bmi160_write_reg(dev, BMI160_CMD, 0xB6); // toggle software reset
    //delay 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 1. acceletrometer */
    bmi160_set_acc_range(dev, BMI160_ACC_RANGE_8G);
    bmi160_write_reg(dev, BMI160_ACC_CONF, 0x2C);  // Set Accel ODR to 1600hz, BWP mode to Oversample 2, acc_us = 0


    // test negative direction
    ESP_LOGD(TAG,"Accel self test sign 0\n");
    uint8_t reg = (0x01 << 0) | (0x00 << 2) | (0x01 << 3); // acc_self_test_en = 1, acc_self_test_sign = 0, acc_self_test_amp = 1
    bmi160_write_reg(dev, BMI160_SELF_TEST, reg);
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t acc_self_test_result;
    bmi160_read_reg(dev, BMI160_STATUS, &acc_self_test_result);

    ESP_LOGD(TAG,"Accel self test result: %02x\n", acc_self_test_result);

    uint8_t rawData[6];
    bmi160_read_reg_array(dev, BMI160_ACC_X_L, rawData, 6);
    float accX0 = (float)((int16_t)(rawData[1] << 8) | rawData[0]) * dev->aRes;
    float accY0 = (float)((int16_t)(rawData[3] << 8) | rawData[2]) * dev->aRes;
    float accZ0 = (float)((int16_t)(rawData[5] << 8) | rawData[4]) * dev->aRes;

    ESP_LOGD(TAG,"Accel self test: %.3f %.3f %.3f\n", accX0, accY0, accZ0);

    // test positive direction
    ESP_LOGD(TAG,"Accel self test sign 1\n");
    reg = (0x01 << 0) | (0x01 << 2) | (0x01 << 3); // acc_self_test_en = 1, acc_self_test_sign = 1, acc_self_test_amp = 1
    bmi160_write_reg(dev, BMI160_SELF_TEST, reg);
    vTaskDelay(pdMS_TO_TICKS(100));
    bmi160_read_reg(dev, BMI160_STATUS, &acc_self_test_result);

    ESP_LOGD(TAG,"Accel self test result: %02x\n", acc_self_test_result);

    
    bmi160_read_reg_array(dev, BMI160_ACC_X_L, rawData, 6);
    float accX1 = (float)((int16_t)(rawData[1] << 8) | rawData[0]) * dev->aRes;
    float accY1 = (float)((int16_t)(rawData[3] << 8) | rawData[2]) * dev->aRes;
    float accZ1 = (float)((int16_t)(rawData[5] << 8) | rawData[4]) * dev->aRes;

    ESP_LOGD(TAG,"Accel self test: %.3f %.3f %.3f\n", accX1, accY1, accZ1);

    ESP_LOGD(TAG,"Accel self test diff: %.3f %.3f %.3f\n", accX1 - accX0, accY1 - accY0, accZ1 - accZ0);


    /* 2. gyroscope */

    bmi160_set_gyr_range(dev, BMI160_GYR_RANGE_1000DPS);
    bmi160_write_reg(dev, BMI160_GYR_CONF, 0x2C);  // Set Gyro ODR to 1600hz, BWP mode to Oversample 2, gyr_us = 0

    bmi160_write_reg(dev, BMI160_SELF_TEST, (uint8_t)(0x1 << 4)); // gyr_self_test_en = 1
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t gyr_self_test_result;
    bmi160_read_reg(dev, BMI160_STATUS, &gyr_self_test_result);
    ESP_LOGD(TAG,"Gyro self test result: %02x\n", gyr_self_test_result);
    if(gyr_self_test_result & (0x1 << 1))
    {
        ESP_LOGD(TAG,"Gyro self test failed\n");
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGD(TAG,"Gyro self test passed\n");
    }

    return ESP_OK;
}

esp_err_t bmi160_enable_int_new_data(bmi160_t *dev, bmi160_int_out_conf_t* intOutConf)
{
        uint8_t data = 0;

        //configure interrupt output
        bmi160_read_reg(dev, BMI160_INT_OUT_CTRL, &data);
        data &= ~(0xfu << (intOutConf->intPin * 4u)); //clear bits
        data |= (uint8_t)(intOutConf->intEnable << ((intOutConf->intPin * 4u) + 3u)); //set enable bit
        data |= (uint8_t)(intOutConf->intOd << ((intOutConf->intPin * 4u) + 2u)); //set open-drain bit
        data |= (uint8_t)(intOutConf->intLevel << ((intOutConf->intPin * 4u) + 1u)); //set active high bit
        bmi160_write_reg(dev, BMI160_INT_OUT_CTRL, data);

        //map interrupt data ready
        if(intOutConf->intPin == BMI160_PIN_INT1)
        {
            data = (1u << 7); //set bit for INT1
        }
        else
        {
            data = (1u << 3); //set bit for INT2
        }
        bmi160_write_reg(dev, BMI160_INT_MAP_1, data); //map data ready interrupt to INT1
        
        //enable interrupt    
        bmi160_read_reg(dev, BMI160_INT_EN_1, &data);
        data |= (1u << 4); // enable data ready interrupt
        bmi160_write_reg(dev, BMI160_INT_EN_1, data);    
    
        return ESP_OK;
}

esp_err_t bmi160_enable_step_counter(bmi160_t *dev, bmi160_step_counter_mode_t mode)
{
    esp_err_t ret = ESP_OK;
    //enable step counter
    bmi160_write_reg(dev, BMI160_PMU_TRIGGER, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t config[2] = {0,0};
    switch (mode)
    {
    case BMI160_STEP_COUNTER_NORMAL:
        config[0] = 0x15;
        config[1] = 0x03;
        break;
    
    case BMI160_STEP_COUNTER_SENSITIVE:
        config[0] = 0x2D;
        config[1] = 0x00;
        break;

    case BMI160_STEP_COUNTER_ROBUST:
        config[0] = 0x1D;
        config[1] = 0x07;
        break;

    default:
        ret = ESP_FAIL;
        break;
    }

    config[1] |= (1u<<3); //enable step counter

    //configure step counter
    ret = bmi160_write_reg_array(dev, BMI160_STEP_CONF_0, config, 2);

    return ret;
}

esp_err_t bmi160_read_step_counter(bmi160_t *dev, uint16_t *stepCounter)
{
    uint8_t data[2];
    bmi160_read_reg_array(dev, BMI160_STEP_CNT_0, data, 2);
    *stepCounter = (uint16_t)((data[1] << 8) | data[0]);

    return ESP_OK;
}

esp_err_t bmi160_reset_step_counter(bmi160_t *dev)
{
    bmi160_write_reg(dev, BMI160_CMD, 0xB2); // reset step counter

    return ESP_OK;
}

esp_err_t bmi160_enable_int_step(bmi160_t *dev, bmi160_int_out_conf_t* intOutConf)
{
    uint8_t data = 0;

    //configure interrupt output
    bmi160_read_reg(dev, BMI160_INT_OUT_CTRL, &data);
    data &= ~(0xfu << (intOutConf->intPin * 4u)); //clear bits
    data |= (uint8_t)(intOutConf->intEnable << ((intOutConf->intPin * 4u) + 3u)); //set enable bit
    data |= (uint8_t)(intOutConf->intOd << ((intOutConf->intPin * 4u) + 2u)); //set open-drain bit
    data |= (uint8_t)(intOutConf->intLevel << ((intOutConf->intPin * 4u) + 1u)); //set active high bit
    bmi160_write_reg(dev, BMI160_INT_OUT_CTRL, data);

    //map interrupt step detection
    data = (1u << 0); //set bit for step detection
    if(intOutConf->intPin == BMI160_PIN_INT1)
    {
        bmi160_write_reg(dev, BMI160_INT_MAP_0, data); //map step detection interrupt to INT1
    }
    else
    {
        bmi160_write_reg(dev, BMI160_INT_MAP_1, data); //map step detection interrupt to INT2
    }
    
    //enable interrupt    
    bmi160_read_reg(dev, BMI160_INT_EN_2, &data);
    data |= (1u << 3); // enable step detection interrupt
    bmi160_write_reg(dev, BMI160_INT_EN_2, data);    

    return ESP_OK;
}


esp_err_t bmi160_switch_mode(bmi160_t *dev, bmi160_pmu_acc_mode_t accMode, bmi160_pmu_gyr_mode_t gyrMode)
{
    //validate parameters
    if(!is_acc_mode_valid(accMode))
    {
        ESP_LOGE(TAG, "Invalid Accelerometer Mode");
        return ESP_FAIL;
    }
    if(!is_gyr_mode_valid(gyrMode))
    {
        ESP_LOGE(TAG, "Invalid Gyroscope Mode");
        return ESP_FAIL;
    }

    uint8_t pmu_status;

    //start up accelerometer
    bmi160_write_reg(dev, BMI160_CMD, accMode);
    vTaskDelay(pdMS_TO_TICKS(100));
    bmi160_read_reg(dev, BMI160_PMU_STATUS, &pmu_status);
    if(((pmu_status & 0x30) >> 4) != (accMode & 0x3))
    {
        ESP_LOGE(TAG, "Accelerometer PMU status: 0x%02x", (pmu_status & 0x30) >> 4);
        return ESP_FAIL;
    }


    //start up gyroscope
    bmi160_write_reg(dev, BMI160_CMD, gyrMode);
    vTaskDelay(pdMS_TO_TICKS(100));
    bmi160_read_reg(dev, BMI160_PMU_STATUS, &pmu_status);
    if(((pmu_status & 0x0C) >> 2) != (gyrMode & 0x3))
    {
        ESP_LOGE(TAG, "Gyroscope PMU status: 0x%02x", (pmu_status & 0x0C) >> 2);
        return ESP_FAIL;
    }
    

    return ESP_OK;
}

esp_err_t bmi160_enable_tap_detection(bmi160_t *dev, bmi160_tap_conf_t* tapConf)
{
    dev->tapMode = tapConf->tapMode;
    uint8_t data = 0;
    data |= (uint8_t)(tapConf->tapQuiet << 7); //set quiet bit
    data |= (uint8_t)(tapConf->tapShock << 6); //set shock bit
    data |= (uint8_t)(tapConf->tapDur << 2); //set duration bits
    bmi160_write_reg(dev, BMI160_INT_TAP_0, data);

    data = (uint8_t)(tapConf->tapTh); //set threshold bits
    bmi160_write_reg(dev, BMI160_INT_TAP_1, data);
    return ESP_OK;
}

esp_err_t bmi160_enable_int_tap(bmi160_t *dev, bmi160_int_out_conf_t* intOutConf)
{
    uint8_t data = 0;

    //configure interrupt output
    bmi160_read_reg(dev, BMI160_INT_OUT_CTRL, &data);
    data &= ~(0xfu << (intOutConf->intPin * 4u)); //clear bits
    data |= (uint8_t)(intOutConf->intEnable << ((intOutConf->intPin * 4u) + 3u)); //set enable bit
    data |= (uint8_t)(intOutConf->intOd << ((intOutConf->intPin * 4u) + 2u)); //set open-drain bit
    data |= (uint8_t)(intOutConf->intLevel << ((intOutConf->intPin * 4u) + 1u)); //set active high bit
    bmi160_write_reg(dev, BMI160_INT_OUT_CTRL, data);

    //map interrupt step detection
    if(dev->tapMode == BMI160_TAP_MODE_SINGLE)
    {
        data = (1u << 5); //set bit for single tap detection
    }
    else
    {
        data = (1u << 4); //set bit for double tap detection
    }
    if(intOutConf->intPin == BMI160_PIN_INT1)
    {
        bmi160_write_reg(dev, BMI160_INT_MAP_0, data); //map step detection interrupt to INT1
    }
    else
    {
        bmi160_write_reg(dev, BMI160_INT_MAP_0, data); //map step detection interrupt to INT2
    }
    
    //enable interrupt    
    bmi160_read_reg(dev, BMI160_INT_EN_0, &data);
    if(dev->tapMode == BMI160_TAP_MODE_SINGLE)
    {
        data |= (1u << 5); //set bit for single tap detection
    }
    else
    {
        data |= (1u << 4); //set bit for double tap detection
    }
    bmi160_write_reg(dev, BMI160_INT_EN_0, data);    

    return ESP_OK;
}

esp_err_t bmi160_read_tap_orient(bmi160_t *dev, uint8_t *orient)
{
    //read int_status_2
    uint8_t data;
    bmi160_read_reg(dev, BMI160_INT_STATUS_2, &data);
    *orient = data;
    return ESP_OK;
}

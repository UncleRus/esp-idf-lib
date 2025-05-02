/**
 * @file husb238.c
 * @brief HUSB238 driver
 * @author xyzroe
 * ESP-IDF driver for HUSB238 USB PD controller
 *
 * Datasheet: https://en.hynetek.com/uploadfiles/site/219/news/b038530d-67c0-4ba0-9269-de0e666cb35b.pdf
 * Registers map: https://en.hynetek.com/uploadfiles/site/219/news/eb6cc420-847e-40ec-a352-a86fbeedd331.pdf
 *
 * Copyright (c) 2025 xyzroe <i@xyzroe.cc>
 * Licensed as described in the file LICENSE
 */

#include "husb238.h"
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>

static const char *TAG = "husb238";

// Additional macros for error checking
#define CHECK_ARG(VAL)                                                                                                                                                                                 \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (!(VAL))                                                                                                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                                                                                                \
    }                                                                                                                                                                                                  \
    while (0)

uint8_t husb238_pd_values[] = {
    0,  // auto mode
    5,  // 5V
    9,  // 9V
    12, // 12V
    15, // 15V
    18, // 18V
    20  // 20V
};

size_t husb238_pd_count = sizeof(husb238_pd_values) / sizeof(husb238_pd_values[0]);

/**
 * @brief Read 8-bit register
 */
static esp_err_t read_reg_8(husb238_t *dev, uint8_t reg, uint8_t *val)
{
    CHECK_ARG(dev && val);
    return i2c_dev_read_reg(&dev->i2c_dev, reg, val, 1);
}

/**
 * @brief Write 8-bit register
 */
static esp_err_t write_reg_8(husb238_t *dev, uint8_t reg, uint8_t val)
{
    CHECK_ARG(dev);
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1);
}

/**
 * @brief Read bits from a register
 */
static esp_err_t read_reg_bits(husb238_t *dev, uint8_t reg, uint8_t bit_start, uint8_t length, uint8_t *data)
{
    CHECK_ARG(dev && data);
    uint8_t val;

    // Read the current value of the register
    ESP_ERROR_CHECK(read_reg_8(dev, reg, &val));

    // Create a mask for length bits starting from bit_start (most significant bit)
    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);

    // Apply the mask and shift the bits to the right to get them in the least significant position
    *data = (val & mask) >> (bit_start - length + 1);

    // Return OK
    return ESP_OK;
}

/**
 * @brief Write bits to a register
 */
static esp_err_t write_reg_bits(husb238_t *dev, uint8_t reg, uint8_t bit_start, uint8_t length, uint8_t data)
{
    CHECK_ARG(dev);
    uint8_t val;

    // Read the current value of the register
    ESP_ERROR_CHECK(read_reg_8(dev, reg, &val));

    // Create a mask for length bits starting from bit_start (most significant bit)
    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);

    // Apply the mask and shift the data to the correct position
    val = (val & ~mask) | ((data << (bit_start - length + 1)) & mask);

    // Write the modified value back to the register
    return write_reg_8(dev, reg, val);
}

esp_err_t husb238_init_desc(husb238_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t addr)
{
    CHECK_ARG(dev);
    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = HUSB238_I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t husb238_free_desc(husb238_t *dev)
{
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t husb238_init(husb238_t *dev)
{
    CHECK_ARG(dev);
    uint8_t reg_val;
    esp_err_t err = read_reg_8(dev, HUSB238_PD_STATUS0, &reg_val);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing HUSB238: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t husb238_is_attached(husb238_t *dev, bool *attached)
{
    CHECK_ARG(dev && attached);
    uint8_t reg_val;
    ESP_ERROR_CHECK(read_reg_bits(dev, HUSB238_PD_STATUS1, 6, 1, &reg_val)); // bits 6
    *attached = (reg_val == 1);
    return ESP_OK;
}

esp_err_t husb238_get_cc_direction(husb238_t *dev, bool *cc_dir)
{
    CHECK_ARG(dev && cc_dir);
    uint8_t reg_val;
    ESP_ERROR_CHECK(read_reg_bits(dev, HUSB238_PD_STATUS1, 7, 1, &reg_val)); // bits 7
    *cc_dir = (reg_val == 1);
    return ESP_OK;
}

esp_err_t husb238_get_pd_response(husb238_t *dev, husb238_response_codes_t *response)
{
    CHECK_ARG(dev && response);
    uint8_t reg_val;
    ESP_ERROR_CHECK(read_reg_bits(dev, HUSB238_PD_STATUS1, 5, 3, &reg_val)); // bits 5,4,3
    *response = (husb238_response_codes_t)reg_val;
    return ESP_OK;
}

esp_err_t husb238_get_5v_contract_voltage(husb238_t *dev, bool *exists)
{
    CHECK_ARG(dev && exists);
    uint8_t reg_val;
    ESP_ERROR_CHECK(read_reg_bits(dev, HUSB238_PD_STATUS1, 2, 1, &reg_val)); // bits 2
    *exists = (reg_val == 1);
    return ESP_OK;
}

esp_err_t husb238_get_5v_contract_current(husb238_t *dev, uint8_t *current)
{
    CHECK_ARG(dev && current);
    uint8_t reg_val;
    ESP_ERROR_CHECK(read_reg_bits(dev, HUSB238_PD_STATUS1, 1, 2, &reg_val)); // bits 1,0
    return convBitsInto5vCur(reg_val, current);
}

esp_err_t husb238_is_voltage_detected(husb238_t *dev, uint8_t voltage, bool *detected)
{
    CHECK_ARG(dev && detected);
    uint8_t reg_val, reg_num;
    ESP_ERROR_CHECK(convVoltIntoPdReg(voltage, &reg_num));
    ESP_ERROR_CHECK(read_reg_bits(dev, reg_num, 7, 1, &reg_val)); // bits 7
    *detected = (reg_val == 1);
    return ESP_OK;
}

esp_err_t husb238_read_detected_current(husb238_t *dev, uint8_t voltage, uint16_t *current)
{
    CHECK_ARG(dev && current);
    uint8_t reg_val, reg_num;

    convVoltIntoPdReg(voltage, &reg_num);
    ESP_ERROR_CHECK(read_reg_bits(dev, reg_num, 3, 4, &reg_val)); // bits 3,2,1,0
    return convCurrentValue(reg_val, current);
}

esp_err_t husb238_get_source_voltage(husb238_t *dev, uint8_t *voltage)
{
    CHECK_ARG(dev && voltage);
    uint8_t reg_val;
    ESP_ERROR_CHECK(read_reg_bits(dev, HUSB238_PD_STATUS0, 7, 4, &reg_val)); // bits 7,6,5,4

    return convBitsIntoVolt(reg_val, voltage);
}

esp_err_t husb238_get_source_current(husb238_t *dev, uint16_t *current)
{
    CHECK_ARG(dev && current);
    uint8_t reg_val;
    ESP_ERROR_CHECK(read_reg_bits(dev, HUSB238_PD_STATUS0, 3, 4, &reg_val)); // bits 3,2,1,0
    return convCurrentValue(reg_val, current);
}

esp_err_t husb238_get_select_pd(husb238_t *dev, uint8_t *pd)
{
    CHECK_ARG(dev && pd);
    uint8_t reg_val;
    ESP_ERROR_CHECK(read_reg_bits(dev, HUSB238_SRC_PDO, 7, 4, &reg_val)); // bits 7,6,5,4
    return convPdBitsIntoVolt(reg_val, pd);
}

esp_err_t husb238_set_select_pd(husb238_t *dev, uint8_t pd)
{
    CHECK_ARG(dev);
    uint8_t reg_val;
    ESP_ERROR_CHECK(convVoltIntoPdBits(pd, &reg_val));
    ESP_ERROR_CHECK(write_reg_bits(dev, HUSB238_SRC_PDO, 7, 4, reg_val)); // bits 7,6,5,4
    return ESP_OK;
}

esp_err_t husb238_reset(husb238_t *dev)
{
    CHECK_ARG(dev);
    uint8_t reg_val = 0b10000;                                               // 0x10
    ESP_ERROR_CHECK(write_reg_bits(dev, HUSB238_GO_COMMAND, 4, 5, reg_val)); // bits 4,3,2,1,0
    return ESP_OK;
}

esp_err_t husb238_request_pd(husb238_t *dev)
{
    CHECK_ARG(dev);
    uint8_t reg_val = 0b00001;                                               // 0x01
    ESP_ERROR_CHECK(write_reg_bits(dev, HUSB238_GO_COMMAND, 1, 2, reg_val)); // bits 1,0
    return ESP_OK;
}

esp_err_t husb238_get_source_capabilities(husb238_t *dev)
{
    CHECK_ARG(dev);
    uint8_t reg_val = 0b00100;                                               // 0x04
    ESP_ERROR_CHECK(write_reg_bits(dev, HUSB238_GO_COMMAND, 4, 5, reg_val)); // bits 4,3,2,1,0
    return ESP_OK;
}

static esp_err_t convCurrentValue(uint8_t reg_val, uint16_t *current)
{
    switch (reg_val)
    {
        case HUSB238_CURRENT_0_5_A:
            *current = 50; // 0.5A
            break;
        case HUSB238_CURRENT_0_7_A:
            *current = 70; // 0.7A
            break;
        case HUSB238_CURRENT_1_0_A:
            *current = 100; // 1.0A
            break;
        case HUSB238_CURRENT_1_25_A:
            *current = 125; // 1.25A
            break;
        case HUSB238_CURRENT_1_5_A:
            *current = 150; // 1.5A
            break;
        case HUSB238_CURRENT_1_75_A:
            *current = 175; // 1.75A
            break;
        case HUSB238_CURRENT_2_0_A:
            *current = 200; // 2.0A
            break;
        case HUSB238_CURRENT_2_25_A:
            *current = 225; // 2.25A
            break;
        case HUSB238_CURRENT_2_50_A:
            *current = 250; // 2.50A
            break;
        case HUSB238_CURRENT_2_75_A:
            *current = 275; // 2.75A
            break;
        case HUSB238_CURRENT_3_0_A:
            *current = 300; // 3.0A
            break;
        case HUSB238_CURRENT_3_25_A:
            *current = 325; // 3.25A
            break;
        case HUSB238_CURRENT_3_5_A:
            *current = 350; // 3.5A
            break;
        case HUSB238_CURRENT_4_0_A:
            *current = 400; // 4.0A
            break;
        case HUSB238_CURRENT_4_5_A:
            *current = 450; // 4.5A
            break;
        case HUSB238_CURRENT_5_0_A:
            *current = 500; // 5.0A
            break;
        default:
            *current = 0; // Unknown value
            return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t convVoltIntoPdReg(uint8_t voltage, uint8_t *reg_val)
{
    switch (voltage)
    {
        case 5:
            *reg_val = HUSB238_SRC_PDO_5V; // SRC_PDO_5V
            break;
        case 9:
            *reg_val = HUSB238_SRC_PDO_9V; // SRC_PDO_9V
            break;
        case 12:
            *reg_val = HUSB238_SRC_PDO_12V; // SRC_PDO_12V
            break;
        case 15:
            *reg_val = HUSB238_SRC_PDO_15V; // SRC_PDO_15V
            break;
        case 18:
            *reg_val = HUSB238_SRC_PDO_18V; // SRC_PDO_18V
            break;
        case 20:
            *reg_val = HUSB238_SRC_PDO_20V; // SRC_PDO_20V
            break;
        default:
            *reg_val = 255; // Unknown value
            return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t convVoltIntoPdBits(uint8_t voltage, uint8_t *reg_val)
{
    switch (voltage)
    {
        case 0:
            *reg_val = HUSB238_PD_NOT_SELECTED; // Unattached
            break;
        case 5:
            *reg_val = HUSB238_PD_SRC_5V; // 5V
            break;
        case 9:
            *reg_val = HUSB238_PD_SRC_9V; // 9V
            break;
        case 12:
            *reg_val = HUSB238_PD_SRC_12V; // 12V
            break;
        case 15:
            *reg_val = HUSB238_PD_SRC_15V; // 15V
            break;
        case 18:
            *reg_val = HUSB238_PD_SRC_18V; // 18V
            break;
        case 20:
            *reg_val = HUSB238_PD_SRC_20V; // 20V
            break;
        default:
            *reg_val = HUSB238_PD_NOT_SELECTED; // Unknown value
            return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t convPdBitsIntoVolt(uint8_t reg_val, uint8_t *pd)
{
    switch (reg_val)
    {
        case HUSB238_PD_NOT_SELECTED:
            *pd = 0; // Unattached
            break;
        case HUSB238_PD_SRC_5V:
            *pd = 5; // 5V
            break;
        case HUSB238_PD_SRC_9V:
            *pd = 9; // 9V
            break;
        case HUSB238_PD_SRC_12V:
            *pd = 12; // 12V
            break;
        case HUSB238_PD_SRC_15V:
            *pd = 15; // 15V
            break;
        case HUSB238_PD_SRC_18V:
            *pd = 18; // 18V
            break;
        case HUSB238_PD_SRC_20V:
            *pd = 20; // 20V
            break;
        default:
            *pd = 255; // Unknown value
            return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t convBitsIntoVolt(uint8_t reg_val, uint8_t *voltage)
{
    switch (reg_val)
    {
        case HUSB238_VOLTAGE_UNATTACHED:
            *voltage = 0; // Unattached
            break;
        case HUSB238_VOLTAGE_5V:
            *voltage = 5; // 5V
            break;
        case HUSB238_VOLTAGE_9V:
            *voltage = 9; // 9V
            break;
        case HUSB238_VOLTAGE_12V:
            *voltage = 12; // 12V
            break;
        case HUSB238_VOLTAGE_15V:
            *voltage = 15; // 15V
            break;
        case HUSB238_VOLTAGE_18V:
            *voltage = 18; // 18V
            break;
        case HUSB238_VOLTAGE_20V:
            *voltage = 20; // 20V
            break;
        default:
            *voltage = 255; // Unknown value
            return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t convBitsInto5vCur(uint8_t reg_val, uint8_t *current)
{
    switch (reg_val)
    {
        case HUSB238_CURRENT_5V_DEFAULT:
            *current = 0; // Default -> 0
            break;
        case HUSB238_CURRENT_5V_1_5_A:
            *current = 15; // 1.5A -> 15
            break;
        case HUSB238_CURRENT_5V_2_4_A:
            *current = 24; // 2.4A -> 24
            break;
        case HUSB238_CURRENT_5V_3_A:
            *current = 30; // 3.0A -> 30
            break;
        default:
            *current = 255; // Unknown value
            return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}
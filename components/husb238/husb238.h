/**
 * @file husb238.h
 * @brief HUSB238 driver header file
 * @author xyzroe
 * ESP-IDF driver for HUSB238 USB PD controller
 *
 * Datasheet: https://en.hynetek.com/uploadfiles/site/219/news/b038530d-67c0-4ba0-9269-de0e666cb35b.pdf
 * Registers map: https://en.hynetek.com/uploadfiles/site/219/news/eb6cc420-847e-40ec-a352-a86fbeedd331.pdf
 *
 * Copyright (c) 2025 xyzroe <i@xyzroe.cc>
 * Licensed as described in the file LICENSE
 */

#ifndef __HUSB238_H__
#define __HUSB238_H__

#include <stdint.h>
#include <stdbool.h>
#include "i2cdev.h"
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HUSB238_I2C_ADDR_DEFAULT 0x08   // Default I2C address
#define HUSB238_I2C_FREQ_HZ      400000 // Max 400 kHz for I2C

// Register addresses
#define HUSB238_PD_STATUS0  0x00
#define HUSB238_PD_STATUS1  0x01
#define HUSB238_SRC_PDO_5V  0x02
#define HUSB238_SRC_PDO_9V  0x03
#define HUSB238_SRC_PDO_12V 0x04
#define HUSB238_SRC_PDO_15V 0x05
#define HUSB238_SRC_PDO_18V 0x06
#define HUSB238_SRC_PDO_20V 0x07
#define HUSB238_SRC_PDO     0x08
#define HUSB238_GO_COMMAND  0x09

// Current values
#define HUSB238_CURRENT_0_5_A  0b0000
#define HUSB238_CURRENT_0_7_A  0b0001
#define HUSB238_CURRENT_1_0_A  0b0010
#define HUSB238_CURRENT_1_25_A 0b0011
#define HUSB238_CURRENT_1_5_A  0b0100
#define HUSB238_CURRENT_1_75_A 0b0101
#define HUSB238_CURRENT_2_0_A  0b0110
#define HUSB238_CURRENT_2_25_A 0b0111
#define HUSB238_CURRENT_2_50_A 0b1000
#define HUSB238_CURRENT_2_75_A 0b1001
#define HUSB238_CURRENT_3_0_A  0b1010
#define HUSB238_CURRENT_3_25_A 0b1011
#define HUSB238_CURRENT_3_5_A  0b1100
#define HUSB238_CURRENT_4_0_A  0b1101
#define HUSB238_CURRENT_4_5_A  0b1110
#define HUSB238_CURRENT_5_0_A  0b1111

// Voltage values
#define HUSB238_VOLTAGE_UNATTACHED 0b0000
#define HUSB238_VOLTAGE_5V         0b0001
#define HUSB238_VOLTAGE_9V         0b0010
#define HUSB238_VOLTAGE_12V        0b0011
#define HUSB238_VOLTAGE_15V        0b0100
#define HUSB238_VOLTAGE_18V        0b0101
#define HUSB238_VOLTAGE_20V        0b0110

// PD selection values
#define HUSB238_PD_NOT_SELECTED 0b0000
#define HUSB238_PD_SRC_5V       0b0001
#define HUSB238_PD_SRC_9V       0b0010
#define HUSB238_PD_SRC_12V      0b0011
#define HUSB238_PD_SRC_15V      0b1000
#define HUSB238_PD_SRC_18V      0b1001
#define HUSB238_PD_SRC_20V      0b1010

// 5V contract current values
#define HUSB238_CURRENT_5V_DEFAULT 0b00
#define HUSB238_CURRENT_5V_1_5_A   0b01
#define HUSB238_CURRENT_5V_2_4_A   0b10
#define HUSB238_CURRENT_5V_3_A     0b11

typedef enum {
    HUSB238_NO_RESPONSE = 0b000,
    HUSB238_SUCCESS = 0b001,
    HUSB238_INVALID_CMD_OR_ARG = 0b011,
    HUSB238_CMD_NOT_SUPPORTED = 0b100,
    HUSB238_TRANSACTION_FAIL_NO_GOOD_CRC = 0b101
} husb238_response_codes_t;

extern uint8_t husb238_pd_values[]; // Array of PD values
extern size_t husb238_pd_count;     // Count of PD values

// Object for HUSB238 device descriptor
typedef struct
{
    i2c_dev_t i2c_dev; // I2C device descriptor
} husb238_t;

/**
 * @brief Initialize device descriptor
 * @param dev Device descriptor
 * @param port I2C port number
 * @param sda_gpio GPIO for SDA
 * @param scl_gpio GPIO for SCL
 * @param addr I2C address
 */
esp_err_t husb238_init_desc(husb238_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t addr);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t husb238_free_desc(husb238_t *dev);

/**
 * @brief Initialize device
 * @param dev Device descriptor
 */
esp_err_t husb238_init(husb238_t *dev);

/**
 * @brief Check if device is attached
 * @param dev Device descriptor
 * @param attached Pointer to store attachment status (true if attached, false otherwise)
 */
esp_err_t husb238_is_attached(husb238_t *dev, bool *attached);

/**
 * @brief Get CC direction
 * @param dev Device descriptor
 * @param cc_dir Pointer to store CC direction (true for CC1 or unattached, false for CC2)
 */
esp_err_t husb238_get_cc_direction(husb238_t *dev, bool *cc_dir);

/**
 * @brief Get PD response
 * @param dev Device descriptor
 * @param response Pointer to store PD response
 */
esp_err_t husb238_get_pd_response(husb238_t *dev, husb238_response_codes_t *response);

/**
 * @brief Get 5V contract voltage
 * @param dev Device descriptor
 * @param exists Pointer to store 5V contract voltage status
 */
esp_err_t husb238_get_5v_contract_voltage(husb238_t *dev, bool *exists);

/**
 * @brief Get 5V contract current
 * @param dev Device descriptor
 * @param current Pointer to store 5V contract current
 */
esp_err_t husb238_get_5v_contract_current(husb238_t *dev, uint8_t *current);

/**
 * @brief check if voltage is detected
 * @param dev Device descriptor
 * @param voltage Voltage to check
 * @param detected Pointer to store voltage detection status
 */
esp_err_t husb238_is_voltage_detected(husb238_t *dev, uint8_t voltage, bool *detected);

/**
 * @brief Read detected current
 * @param dev Device descriptor
 * @param voltage Voltage to read current for
 * @param current Pointer to store detected current
 */
esp_err_t husb238_read_detected_current(husb238_t *dev, uint8_t voltage, uint16_t *current);

/**
 * @brief Get source voltage
 * @param dev Device descriptor
 * @param voltage Pointer to store source voltage
 */
esp_err_t husb238_get_source_voltage(husb238_t *dev, uint8_t *voltage);

/**
 * @brief Get source current
 * @param dev Device descriptor
 * @param current Pointer to store source current
 */
esp_err_t husb238_get_source_current(husb238_t *dev, uint16_t *current);

/**
 * @brief get selected PD
 * @param dev Device descriptor
 * @param pd Pointer to store selected PD
 */
esp_err_t husb238_get_select_pd(husb238_t *dev, uint8_t *pd);

/**
 * @brief Select PD
 * @param dev Device descriptor
 * @param pd PD selection
 */
esp_err_t husb238_set_select_pd(husb238_t *dev, uint8_t pd);

/**
 * @brief Reset HUSB238
 * @param dev Device descriptor
 */
esp_err_t husb238_reset(husb238_t *dev);

/**
 * @brief Request PD
 * @param dev Device descriptor
 */
esp_err_t husb238_request_pd(husb238_t *dev);

/**
 * @brief get source capabilities. seems that it brakes the PD selection
 * @param dev Device descriptor
 */
esp_err_t husb238_get_source_capabilities(husb238_t *dev);

/**
 * @brief Convert current value to register value
 * @param reg_val Register value
 * @param current Pointer to store current value
 */
static esp_err_t convCurrentValue(uint8_t reg_val, uint16_t *current);

/**
 * @brief Convert voltage value to PD register value
 * @param voltage Voltage value
 * @param reg_val Pointer to store register value
 */
static esp_err_t convVoltIntoPdReg(uint8_t voltage, uint8_t *reg_val);

/**
 * @brief Convert voltage value to PD bits
 * @param voltage Voltage value
 * @param reg_val Pointer to store register value
 */
static esp_err_t convVoltIntoPdBits(uint8_t voltage, uint8_t *reg_val);

/**
 * @brief Convert PD register value to voltage value
 * @param reg_val Register value
 * @param pd Pointer to store voltage value
 */
static esp_err_t convPdBitsIntoVolt(uint8_t reg_val, uint8_t *pd);

/**
 * @brief Convert bits value to voltage value
 * @param reg_val Register value
 * @param voltage Pointer to store voltage value
 */
static esp_err_t convBitsIntoVolt(uint8_t reg_val, uint8_t *voltage);

/**
 * @brief Convert bits value to 5V current value
 * @param reg_val Register value
 * @param current Pointer to store current value
 */
static esp_err_t convBitsInto5vCur(uint8_t reg_val, uint8_t *current);

#ifdef __cplusplus
}
#endif

#endif // __HUSB238_H__
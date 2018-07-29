/**
 * @file mcp23017.h
 *
 * ESP-IDF driver for I2C 16 bit GPIO expander MCP23017
 *
 * Copyright (C) 2018 Ruslan V. Uss (https://github.com/UncleRus)
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP23017_H__
#define __MCP23017_H__

#include <stddef.h>
#include <stdbool.h>
#include <i2cdev.h>

#define MCP23017_I2C_ADDR_BASE 0x20

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MCP23017_GPIO_OUTPUT = 0,
    MCP23017_GPIO_INPUT
} mcp23017_gpio_mode_t;

typedef enum
{
    MCP23017_ACTIVE_LOW = 0,
    MCP23017_ACTIVE_HIGH,
    MCP23017_OPEN_DRAIN
} mcp23017_int_out_mode_t;

typedef enum
{
    MCP23017_INT_DISABLED = 0,
    MCP23017_INT_LOW_EDGE,
    MCP23017_INT_HIGH_EDGE,
    MCP23017_INT_ANY_EDGE
} mcp23017_gpio_intr_t;

/**
 * @brief Initialize device descriptior
 * SCL frequency is 1MHz
 * @param dev Pointer to I2C device descriptor
 * @param port I2C port number
 * @param addr I2C address (`0b0100<A2><A1><A0>`)
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_free_desc(i2c_dev_t *dev);

/**
 * Get INTA/INTB pins mode
 * @param dev Pointer to I2C device descriptor
 * @param[out] mode Buffer to store mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_get_int_out_mode(i2c_dev_t *dev, mcp23017_int_out_mode_t *mode);

/**
 * Set INTA/INTB pins mode
 * @param dev Pointer to I2C device descriptor
 * @param mode INTA/INTB pins mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_set_int_out_mode(i2c_dev_t *dev, mcp23017_int_out_mode_t mode);

esp_err_t mcp23017_port_get_mode(i2c_dev_t *dev, uint16_t *val);

/**
 * @brief Set GPIO pins mode
 * 0 - output, 1 - input for each bit in `val`
 * @param dev Pointer to I2C device descriptor
 * @param val Mode, 0 bit for PORTA/GPIO0 .. 15 bit for PORTB/GPIO7
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_port_set_mode(i2c_dev_t *dev, uint16_t val);

esp_err_t mcp23017_port_get_pullup(i2c_dev_t *dev, uint16_t *val);
esp_err_t mcp23017_port_set_pullup(i2c_dev_t *dev, uint16_t val);

/**
 * @brief Read GPIO port value
 * @param dev Pointer to I2C device descriptor
 * @param[out] val 16-bit GPIO port value
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_port_read(const i2c_dev_t *dev, uint16_t *val);

/**
 * @brief Write value to GPIO port
 * @param dev Pointer to I2C device descriptor
 * @param value GPIO port val
 * @return ESP_OK on success
 */
esp_err_t mcp23017_port_write(const i2c_dev_t *dev, uint16_t val);

esp_err_t mcp23017_get_mode(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_mode_t *mode);
esp_err_t mcp23017_set_mode(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_mode_t mode);

esp_err_t mcp23017_get_pullup(i2c_dev_t *dev, uint8_t pin, bool *enable);
esp_err_t mcp23017_set_pullup(i2c_dev_t *dev, uint8_t pin, bool enable);

esp_err_t mcp23017_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t val);
esp_err_t mcp23017_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val);

esp_err_t mcp23017_set_interrupt(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_intr_t intr);
esp_err_t mcp23017_port_set_interrupt(i2c_dev_t *dev, uint16_t mask, mcp23017_gpio_intr_t intr);


#ifdef __cplusplus
}
#endif

#endif /* __MCP23017_H__ */

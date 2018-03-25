#ifndef __I2C_UTILS_H__
#define __I2C_UTILS_H__

#include <driver/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t i2c_setup_master(i2c_port_t i2c_num, gpio_num_t scl_pin, gpio_num_t sda_pin, uint32_t clk_freq);

esp_err_t i2c_read_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, void *res, size_t size);

esp_err_t i2c_write_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, void *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_UTILS_H__ */

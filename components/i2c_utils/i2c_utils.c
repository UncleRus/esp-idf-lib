#include "i2c_utils.h"

#include <freertos/FreeRTOS.h>

esp_err_t i2c_write_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, void *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, CONFIG_I2C_TIMEOUT / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t i2c_read_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, void *res, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | 1, true);
    i2c_master_read(cmd, res, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, CONFIG_I2C_TIMEOUT / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t i2c_setup_master(i2c_port_t i2c_num, gpio_num_t scl_pin, gpio_num_t sda_pin, uint32_t clk_freq)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk_freq;

    esp_err_t res = i2c_param_config(i2c_num, &conf);
    if (res != ESP_OK)
        return res;
    return i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
}

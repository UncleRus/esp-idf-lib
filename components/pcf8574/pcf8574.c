#include "pcf8574.h"
#include <esp_err.h>

#define CLOCK_FREQ_HZ 100000

esp_err_t pcf8574_init_desc(i2c_dev_t *dev, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
    dev->cfg.master.clk_speed = CLOCK_FREQ_HZ;

    return ESP_OK;
}

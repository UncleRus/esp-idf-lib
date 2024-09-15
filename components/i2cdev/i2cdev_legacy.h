#if !defined __I2CDEV_LEGACY_H__
#define __I2CDEV_LEGACY_H__

#include <esp_err.h>
#include "i2cdev.h"

esp_err_t _i2cdev_init();

esp_err_t _i2cdev_done();

esp_err_t _i2c_dev_probe();

esp_err_t _i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size);

esp_err_t _i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size);

#endif // __I2CDEV_LEGACY_H__

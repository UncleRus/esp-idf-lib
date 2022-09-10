

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2c_dev_bridge.h"
#include "i2cdev.h"

#define TAG "i2c dev bridge"
/* ----------------------------------------------------- */
uint8_t mpu_6050_get_device_addr(void)
{
    esp_err_t ret = ESP_OK;
    uint8_t address;
    bool status = false;
    for (int i = 0; i < 128; i += 16)
    {
        ESP_LOGD(TAG, "%02x: ", i);
        for (int j = 0; j < 16; j++)
        {
            address = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
        }
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get the device addrees.");
        address = I2C_DEVICE_ADDR_NONE;
    }

    return address;
}
esp_err_t esp32_i2c_read_bytes(const i2c_dev_t *dev, uint8_t reg_addr, size_t size, uint8_t *data)
{
    return i2c_dev_read(dev, &(reg_addr), sizeof(uint8_t), data, size);
}
/* ----------------------------------------------------- */
esp_err_t esp32_i2c_read_byte(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t *data)
{
    return (i2c_dev_read_reg(dev, reg_addr, (void *)data, 1));
}

/* ----------------------------------------------------- */
esp_err_t esp32_i2c_read_bits(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_start, uint8_t size, uint8_t *data)
{
    esp_err_t ret = ESP_OK;
    uint8_t bit;
    ret = esp32_i2c_read_byte(dev, reg_addr, &(bit));
    if (ret == ESP_OK)
    {
        uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);

        bit &= mask;
        bit >>= (bit_start - size + 1);
        *data = bit;
    }
    return ret;
}
/* ----------------------------------------------------- */
esp_err_t esp32_i2c_read_bit(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_number, uint8_t *data)
{
    uint8_t bit;
    ESP_ERROR_CHECK(esp32_i2c_read_byte(dev, reg_addr, &bit));
    *data = bit & (1 << bit_number);
    return ESP_OK;
}
esp_err_t esp32_i2c_write_bytes(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t size)
{
    return i2c_dev_write(dev, &(reg_addr), sizeof(reg_addr), data, size);
}
/* ----------------------------------------------------- */
esp_err_t esp32_i2c_write_byte(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t data)
{
    return (i2c_dev_write_reg(dev, reg_addr, &(data), 1));
}
/* ----------------------------------------------------- */
esp_err_t esp32_i2c_write_bits(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_start, uint8_t size, uint8_t data)
{
    esp_err_t ret = ESP_OK;
    uint8_t bit = 0;
    ret = esp32_i2c_read_byte(dev, reg_addr, &bit);
    if (ret == ESP_OK)
    {
        uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);
        data <<= (bit_start - size + 1);
        data &= mask;
        bit &= ~(mask);
        bit |= data;
        return (esp32_i2c_write_byte(dev, reg_addr, bit));
    }

    return ret;
}
/* ----------------------------------------------------- */
esp_err_t esp32_i2c_write_bit(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_number, uint8_t data)
{
    esp_err_t ret = ESP_OK;
    uint8_t bit;

    ret = esp32_i2c_read_byte(dev, reg_addr, &bit);

    if (data != 0)
    {
        bit = (bit | (1 << bit_number));
    }
    else
    {
        bit = (bit & ~(1 << bit_number));
    }
    ret = esp32_i2c_write_byte(dev, reg_addr, bit);
    return ret;
}
/* ----------------------------------------------------- */
esp_err_t esp32_i2c_write_word(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t data)
{
    esp_err_t ret = ESP_OK;
    uint8_t data_1[] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };

    ret = esp32_i2c_write_bytes(dev, reg_addr, (uint8_t *)&(data_1), 2);

    return ret;
}
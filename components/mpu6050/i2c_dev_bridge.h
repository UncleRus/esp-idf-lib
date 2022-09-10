
#ifndef I2C_DEV_BRIDGE_H
#define I2C_DEV_BRIDGE_H

#include <i2cdev.h>

#define I2C_DEVICE_ADDR_NONE ((uint8_t)0x00)

/**
 * @brief
 *
 * @return
 *      - address of the sensor
 *      - I2c_DEVICE_ADDR_NONE.
 */
uint8_t mpu_6050_get_device_addr(void);

/**
 * @brief Read bytes from register.
 *
 * @param dev i2c dev config struct.
 * @param reg_addr: register address
 * @param size size of buffer
 * @param data  Buffer to store the read data in.
 * @return
 *      - ESP_OK if able to read else
 *      - ESP_FAIL
 */
esp_err_t esp32_i2c_read_bytes(const i2c_dev_t *dev, uint8_t reg_addr, size_t size, uint8_t *data);

/**
 * @brief Read single byte from an 8-bit register.
 *
 * @param dev i2c dev config struct.
 * @param reg_addr Address of the first register to read from.
 * @param data Buffer to store the read data in.
 * @return
 *      - ESP_OK if able to read else
 *      - ESP_FAIL
 *
 */
esp_err_t esp32_i2c_read_byte(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t *data);

/**
 * @brief read bit from register
 * 
 * @param dev i2c dev config struct.
 * @param reg_addr register address
 * @param bit_number postition of the bit which want to read
 * @param data Buffer to store the read data in.
 * @return
 *      - ESP_OK if able to read else
 *      - ESP_FAIL
 */
esp_err_t esp32_i2c_read_bit(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_number, uint8_t *data);

/**
 * @brief Read multiple bits from an 8-bit register.
 *
 * @param dev i2c dev config struct.
 * @param reg_addr Address of the register to read from.
 * @param bit_start first bit position to read (0-7).
 * @param size Container to store the right-aligned value.
 * @param data Number of bits to read (Not more than 8).
 * @return
 *      - ESP_OK if able to read
 */
esp_err_t esp32_i2c_read_bits(const i2c_dev_t *dev, uint8_t reg_addr,
                              uint8_t bit_start, uint8_t size, uint8_t *data);
/**
 * @brief Write single byte to an 8-bit register.
 *
 * @param dev i2c dev config struct.
 * @param reg_addr Address of the register to write to.
 * @param data Array of bytes to write.
 * @return
 *      - ESP_OK if able to write
 */
esp_err_t esp32_i2c_write_byte(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t data);

/**
 * @brief Write multiple bits to an 8-bit register.
 *
 * @param dev i2c dev config struct.
 * @param reg_addr Address of the register to write to.
 * @param bit_start First bit position to write (0-7).
 * @param size Number of bits to write (Not more than 8).
 * @param data Right-aligned value to write.
 * @return
 *      - ESP_OK if able to write
 */
esp_err_t esp32_i2c_write_bits(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_start, uint8_t size, uint8_t data);

/**
 * @brief Write single bit to an 8-bit register.
 *
 * @param devi2c dev config struct.
 * @param reg_addr Address of the register to write to.
 * @param bit_number Bit position to write (0-7).
 * @param data Bit value to write.
 * @return
 *      - ESP_OK if able to write
 */
esp_err_t esp32_i2c_write_bit(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t bit_number, uint8_t data);

/**
 * @brief write 8-bit data
 * 
 * @param dev i2c dev config struct.
 * @param reg_addr Address of the register to write to.
 * @param data data to write
 * @return
 *      - ESP_OK if able to write
 */
esp_err_t esp32_i2c_write_word(const i2c_dev_t *dev, uint8_t reg_addr, uint8_t data);

#endif /* I2C_DEV_BRIDGE_H */
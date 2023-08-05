# Porting I2C libs to I2Cdev

The synchronization mechanism in i2cdev is achieved with mutexes. Each of two I2C harware ports is protected with a mutex hidden from the user. And yet the user is required to wrap all I2C commands with a separate (per-component) mutex that ensures that all I2C transactions are executed in correct order.

In other words, making an I2C lib to be I2Cdev-friendly requires the user to

* Initialize the mutex with `i2c_dev_create_mutex`.
* Call `i2c_dev_take_mutex` and `i2c_dev_give_mutex` at the beginning and the end of a function.

Still, this is not enough because it does not exclude scenarios when another i2cdev component accesses the I2C periphery in its thread. Therefore, all I2C raw calls must be rewritten to `i2c_dev_xxx` functions.

<details>

<summary>Example 1. Porting ssd1306 OLED lib</summary>

Original repository: https://github.com/TaraHoleInIt/tarablessd1306

The original file `default_if_i2c.c` could be rewritten to use `i2cdev` commands like this:

```c
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include "ssd1306.h"
#include "ssd1306_default_if.h"

#include "i2cdev.h"
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>

#define SSD1306_I2C_COMMAND_MODE  0x00
#define SSD1306_I2C_DATA_MODE     0x40
#define SSD1360_I2C_ADDRESS       0x3C


static i2c_dev_t dev;


esp_err_t SSD1306_I2CMasterInitDefault(i2c_port_t port, gpio_num_t sda_gpio,
                                       gpio_num_t scl_gpio) {
  memset(&dev, 0, sizeof(i2c_dev_t));
  dev.port = port;
  dev.addr = SSD1360_I2C_ADDRESS;
  dev.cfg.sda_io_num = sda_gpio;
  dev.cfg.scl_io_num = scl_gpio;
  dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  dev.cfg.master.clk_speed = 1000000;
  esp_err_t err = i2c_dev_create_mutex(&dev);
  return err;
}

void SSD1306_Done() {
    i2c_dev_delete_mutex(&dev);
}

bool SSD1306_IsDisplayAttached() {
    static const uint8_t Data[] = {0};
    if (i2c_dev_take_mutex(&dev) != ESP_OK) {
        return false;
    }
    esp_err_t err = i2c_dev_write_reg(&dev, SSD1306_I2C_COMMAND_MODE, Data, 1);
    i2c_dev_give_mutex(&dev);
    return err == ESP_OK;
}

bool SSD1306_I2CMasterAttachDisplayDefault(struct SSD1306_Device *DisplayHandle,
                                           int Width, int Height, int RSTPin) {
  NullCheck(DisplayHandle, return false);
  return SSD1306_Init_I2C(DisplayHandle, Width, Height, RSTPin, I2CDefaultWriteCommand, I2CDefaultWriteData,
                          I2CDefaultReset);
}

static bool I2CDefaultWriteBytes(int Address, bool IsCommand,
                                 const uint8_t *Data, size_t DataLength) {

  NullCheck(Data, return false);
  if (i2c_dev_take_mutex(&dev) != ESP_OK) {
      return false;
  }
  uint8_t reg_mode = IsCommand ? SSD1306_I2C_COMMAND_MODE : SSD1306_I2C_DATA_MODE;
  esp_err_t err = i2c_dev_write_reg(&dev, reg_mode, Data, DataLength);
  i2c_dev_give_mutex(&dev);
  return err == ESP_OK;
}
```

</details>

<details>

<summary>Example 2. Porting at24c EEPROM lib</summary>

Original repository: https://github.com/nopnop2002/esp-idf-24c

Let's add an instance of `i2c_dev_t` to the already defined structure:

```c
typedef struct {
    i2c_dev_t i2c_dev;      /* I2C thread-safe dev. */
    ...
} EEPROM_t;

```

### Initialization

```c
void eeprom_init(EEPROM_t *eeprom_dev, i2c_port_t i2c_port, int sda_gpio,
        int scl_gpio, int16_t size, int chip_addr) {
    memset(eeprom_dev, 0, sizeof(EEPROM_t));
    eeprom_dev->addr0 = chip_addr;
    eeprom_dev->size = size;
    eeprom_dev->bytes = 128 * size;

    eeprom_dev->i2c_dev.port = i2c_port;
    eeprom_dev->i2c_dev.addr = chip_addr;
    eeprom_dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    eeprom_dev->i2c_dev.cfg.scl_io_num = scl_gpio;
    eeprom_dev->i2c_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    eeprom_dev->i2c_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    eeprom_dev->i2c_dev.cfg.master.clk_speed = 400000;
    i2c_dev_create_mutex(&eeprom_dev->i2c_dev);
}

void eeprom_done(EEPROM_t *dev) {
    // release the resources if
    i2c_dev_delete_mutex(&dev->i2c_dev);
}
```

### I2C read and write commands

#### ReadReg8

```c
/* Original */
static esp_err_t ReadReg8(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint8_t data_addr, uint8_t * data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data, NACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}


/* Thread-safe */
static esp_err_t ReadReg8(EEPROM_t *dev, int chip_addr, uint8_t data_addr,
        uint8_t *data) {
    esp_err_t err = i2c_dev_take_mutex(&dev->i2c_dev);
    if (err != ESP_OK) {
        return err;
    }
    dev->i2c_dev.addr = chip_addr;
    err = i2c_dev_read_reg(&dev->i2c_dev, data_addr, data, 1);
    i2c_dev_give_mutex(&dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ReadReg8 %s", esp_err_to_name(err));
    }
    return err;
}
```

Note that the `i2c_dev.addr` is changed on each call of `ReadReg8` because `i2c_dev_xxx` functions rely on the `addr` member of the `i2c_dev_t` struct.


#### WriteReg8

```c
/* Original */
static esp_err_t WriteReg8(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint8_t data_addr, uint8_t data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	usleep(1000*2);
	return ret;
}


/* Thread-safe */
static esp_err_t WriteReg8(EEPROM_t *dev, int chip_addr, uint8_t data_addr,
        uint8_t data) {
    esp_err_t err = i2c_dev_take_mutex(&dev->i2c_dev);
    if (err != ESP_OK) {
        return err;
    }
    dev->i2c_dev.addr = chip_addr;
    err = i2c_dev_write_reg(&dev->i2c_dev, data_addr, &data, 1);
    i2c_dev_give_mutex(&dev->i2c_dev);
    usleep(1000 * 2);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WriteReg8 %s", esp_err_to_name(err));
    }
    return err;
}
```


#### ReadReg16

```c
/* Original */
static esp_err_t ReadReg16(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint16_t data_addr, uint8_t * data)
{
	uint8_t high_addr = (data_addr >> 8) & 0xff;
	uint8_t low_addr = data_addr & 0xff;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, high_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, low_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data, NACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}


/* Thread-safe */
static esp_err_t ReadReg16(EEPROM_t *dev, uint16_t data_addr, uint8_t *data) {
    esp_err_t err = i2c_dev_take_mutex(&dev->i2c_dev);
    if (err != ESP_OK) {
        return err;
    }
    dev->i2c_dev.addr = dev->addr0;
    uint8_t high_addr = (data_addr >> 8) & 0xff;
    uint8_t low_addr = data_addr & 0xff;
    uint8_t addr16[] = { high_addr, low_addr };
    err = i2c_dev_read(&dev->i2c_dev, addr16, 2, data, 1);
    i2c_dev_give_mutex(&dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ReadReg16 %s", esp_err_to_name(err));
    }
    return err;
}
```


#### WriteReg16


```c
/* Original */
static esp_err_t WriteReg16(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint16_t data_addr, uint8_t data)
{
	uint8_t high_addr = (data_addr >> 8) & 0xff;
	uint8_t low_addr = data_addr & 0xff;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, high_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, low_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	usleep(1000*2);
	return ret;
}


/* Thread-safe */
static esp_err_t WriteReg16(EEPROM_t *dev, uint16_t data_addr, uint8_t data) {
    esp_err_t err = i2c_dev_take_mutex(&dev->i2c_dev);
    if (err != ESP_OK) {
        return err;
    }
    dev->i2c_dev.addr = dev->addr0;
    uint8_t high_addr = (data_addr >> 8) & 0xff;
    uint8_t low_addr = data_addr & 0xff;
    uint8_t addr16[] = { high_addr, low_addr };
    err = i2c_dev_write(&dev->i2c_dev, addr16, 2, &data, 1);
    i2c_dev_give_mutex(&dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WriteReg16 %s", esp_err_to_name(err));
    }
    return err;
}
```

`ReadReg16` and `WriteReg16` functions demostrate how to pack several consecutive executions of `i2c_master_write_byte` into an array of bytes and call `i2c_dev_read` or `i2c_dev_write` respectively.


### Usage

```c
void app_main(void)
{

    i2cdev_init();  // must be called first
    // initialize all i2cdev components here
    SSD1306_I2CMasterInitDefault(...);
    eeprom_init(...);
}
```

</details>

You may find [this](https://github.com/UncleRus/esp-idf-lib/discussions/367) discussion useful.
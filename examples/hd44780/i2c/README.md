# Example for `hd44780` and `pcf8574` driver

An example to use `HD44780` LCD with `PCF8574` Remote 8-Bit I/O Expander for
I2C Bus.

## What it does

It prints "Hello world!" with two user-defined characters.

It prints current time in a loop.

## Wiring

In most cases, the `HD44780` module comes with a `PCF8574` module designed for
`HD44780`. The `PCF8574` module has pins for I2C and `hd44780`. In that case,
you simply connect two modules. The example assumes a typical configuration,
described below.

| `hd44780` | `PCF8574` |
|-----------|-----------|
| `RS`      | 0         |
| `E`       | 2         |
| `D4`      | 4         |
| `D5`      | 5         |
| `D6`      | 6         |
| `D7`      | 7         |
| `BL`      | 3         |

![Module schematics](i2c_lcd.png?raw=true)

Connect `SCL` and `SDA` pins on `PCF8574` module to the following pins with
appropriate pull-up resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

The default I2C address of `PCF8574` is `0x27`, which can be modified under
`Example configuration` in `menuconfig`.

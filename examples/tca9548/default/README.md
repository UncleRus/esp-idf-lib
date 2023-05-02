# Example for `tca9548` driver

## What it does

It shows temperature and pressure measured by two BMP180 connected to TCA9548.

## Wiring

- Connect `SCL` and `SDA` pins of TCA9548 to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

- Connect `SCL` and `SDA` pins of first BMP180 to the `SC0` and `SD0` of TCA9548 
- Connect `SCL` and `SDA` pins of second BMP180 to the `SC1` and `SD1` of TCA9548

## Notes

- Choose correct I2C address of TCA9548 in `menuconfig`. It's 0x70 by default.
- `CONFIG_NEWLIB_LIBRARY_LEVEL_NORMAL` must be `y` on `esp8266`.

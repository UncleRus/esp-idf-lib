# Example for `bmi160` driver default use

## What it does

Example configures a BMI160 device.

It shows step counter in a loop.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_INT1` | GPIO number for `INT1` | "2" for `esp8266`, "5" for `esp32c3`, "2" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes
- Choose I2C address under `Example configuration` in `menuconfig`. The default is
  `BMI160_I2C_ADDRESS_VDD` (`ADDR` pin is connected to VDD).

# Example for `scd30` driver

## What it does

Reads temperature, relative humidity, and CO2 concentration in a loop and logs them.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

Set `CONFIG_EXAMPLE_I2C_MASTER_SCL` and `CONFIG_EXAMPLE_I2C_MASTER_SDA` under `Example configuration` in `menuconfig` if different than defaults above.
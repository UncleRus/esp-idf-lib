# Example for `aht` driver

## What it does

It checks sensor calibration state and then shows temperature and humidity in loop.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

- Choose I2C address under `Example configuration` in `menuconfig`. The default is
  `AHT_I2C_ADDRESS_GND` (`ADDR` pin is connected to GND).
- Choose chip type under `Example configuration` in `menuconfig`. The default is
  `AHT_TYPE_AHT1x` (AHT10 or AHT15)

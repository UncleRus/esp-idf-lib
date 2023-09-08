# Example for `ina260` driver

## Datasheet

[INA260](https://www.ti.com/lit/ds/symlink/ina260.pdf)

## What it does

The example configures a `ina260` device (triggered mode, 128 samples, 1.1ms
conversion time).

It shows voltage, current, and power in a loop.

## Wiring

The example uses `0x40` as I2C address by default (`A0` and `A1` are grounded).
Use `menuconfig` to change it.

Connect `IN+` and `IN-` to the load.

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

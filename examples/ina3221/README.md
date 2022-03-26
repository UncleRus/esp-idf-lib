# Example for `ina3221` driver

## What it does

It configures the device; enable all three channels, use 64 samples average,
set conversion time to 2 ms, and set warning current threshold  at
`WARNING_CURRENT`, or 40.0 (hard-coded) mA, on `WARNING_CHANNEL`, or channel
`1`.

It shows bus voltage, shunt voltage, and shunt current in a loop.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

Shunt resistors are assumed to be 100 mOhm. Change the value of
`CONFIG_EXAMPLE_SHUNT_RESISTOR_MILLI_OHM` under `Example configuration` in
`menuconfig` if they are not.

The default measuring mode is "triggered". It can be modified under `Example
configuration` in `menuconfig`.

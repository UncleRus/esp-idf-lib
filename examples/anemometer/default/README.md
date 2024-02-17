# Example for `anemometer` driver

## What it does

It shows anemometer impulse wind sensor sensor data in a loop.

## Wiring

Connect `DATA` pin to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_ANEMOMETER_GPIO` | GPIO number for `DATA` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

`CONFIG_NEWLIB_LIBRARY_LEVEL_NORMAL` must be `y` on `esp8266`.

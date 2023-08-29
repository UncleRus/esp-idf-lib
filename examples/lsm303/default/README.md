# Example for `lsm303` driver

## What it does

It shows LSM303: 3-axis accelerometer and magnetometer sensor data in a loop.

## Wiring

Connect `DATA` pin to the following pin:

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_DATA_GPIO` | GPIO number for `DATA` pin | "4" for `esp8266` and `esp32c3`, "17" for `esp32`, `esp32s2`, and `esp32s3` |


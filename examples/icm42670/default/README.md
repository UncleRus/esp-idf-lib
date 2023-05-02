# Example for `icm42670` driver

## What it does

This example shows the basic configuration and initialization of the ICM42670 IMU and prints raw accelerometer and gyro values.

Optional configuration of output data rate (ODR), full scale range (FSR) and low pass filters (LPF) is shown.


## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "8" (`esp32c3`-based ESP-RS board), "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "10" (`esp32c3`-based ESP-RS board), "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

Choose I2C address under `Example configuration` in `menuconfig`. The default is
`ICM42670_I2C_ADDR_GND` (0x68).

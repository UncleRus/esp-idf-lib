# Example for `lis2hh12` driver

## What it does

First, runs the device self test procedure.

If successful, starts reading the acceleration data in polling mode.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_LIS2HH12_SA0_PIN` | Level of the pin `SA0` on the LIS2HH12 | not selected (LOW) |

## Based on

This code is based on the examples `lis2hh12_self_test.c` and
`lis2hh12_data_polling.c` by ST Microelectronics. They can be found here:
https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lis2hh12_STdC/examples
# Example for `max31865` driver

An example to use `max31865`.

## What it does

It measures and shows temperature of RTD in a loop.

## Wiring

Connect `MOSI`, `MISO`, `SCLK` and `CS` pins to the following pins:

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_MOSI_GPIO` | GPIO number for `MOSI` | "4" for `esp32c3`, "23" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_MISO_GPIO` | GPIO number for `MISO` | "5" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_SCLK_GPIO` | GPIO number for `SCLK` | "6" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_CS_GPIO` | GPIO number for `CS` | "7" for `esp32c3`, "5" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

Please configure parameters of your RTD and its connection to MAX31865
under `Example configuration` in `menuconfig`.

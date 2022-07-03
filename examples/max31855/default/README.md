# Example for `max31855` driver

An example to use `max31855`.

## What it does

It measures and shows temperature and state of the thermocouple in a loop.

## Wiring

Connect `MISO`, `SCLK` and `CS` pins to the following pins:

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_MISO_GPIO` | GPIO number for `MISO` | "5" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_SCLK_GPIO` | GPIO number for `SCLK` | "6" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_CS_GPIO` | GPIO number for `CS` | "7" for `esp32c3`, "5" for `esp32`, `esp32s2`, and `esp32s3` |


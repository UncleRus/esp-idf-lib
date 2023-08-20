# Example for `ds1302` driver

## What it does

It initializes the date and the time to 2018-03-11 00:52:10.

It shows current date and time in a loop.

## Wiring

Connect `CE`, `I/O` and `SCLK` pins to the following GPIO with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_CE_GPIO` | GPIO number for `CE` | "5" for `esp8266` and `esp32c3`, "16" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_IO_GPIO` | GPIO number for `I/O` | "4" for `esp8266` and `esp32c3`, "17" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_SCLK_GPIO` | GPIO number for `SCLK` | "0" for `esp8266` and `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

# Example for `mhz19b` driver

## What it does

The example connects to a `MHZ-19` device over UART. It shows the firmware
version, configures it, waits for it to warm-up.

It shows CO2 value from the sensor in a loop.

## Wiring

Connect UART `TX` pin of `MHZ-19` to `CONFIG_EXAMPLE_UART_RX` pin (see the
defaults for each supported target).

Connect UART `RX` pin of `MHZ-19` to `CONFIG_EXAMPLE_UART_TX` pin (see the
defaults for each supported target).

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_UART_TX` | GPIO number for `TX` | "15" for `esp8266`, "18" for `esp32c3`, "12" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_UART_RX` | GPIO number for `RX` | "13" for `esp8266`, "19" for `esp32c3`, "13" for `esp32`, `esp32s2`, and `esp32s3` |

Change the defaults GPIO numbers under `Example configuration` by `make
menuconfig`.

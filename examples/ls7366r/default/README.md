# Example for `ls7366r` driver

## Datasheet

[LS7366R](https://www.lsicsi.com/datasheets/LS7366R.pdf)

## What it does

The example configures a `ls7366r` device on a SPI bus. It creates a task to
toggle `CONFIG_EXAMPLE_PIN_NUM_TEST` pin, and configures an interrupt handler
to receive interrupts.

It reads the counter value from the device in a loop.

## Wiring

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_PIN_NUM_MOSI` | GPIO number for `MOSI` | "13" for `esp8266`, "23" for `esp32c3`, `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_PIN_NUM_MISO` | GPIO number for `MISO` | "12" for `esp8266`, "19" for `esp32c3`, `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_PIN_NUM_CLK`  | GPIO number for `CLK`  | "14" for `esp8266`, "18" for `esp32c3`, `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_PIN_CS`       | GPIO number for `CS` | "15 for `esp8266`, "5" for `esp32c3`, `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_PIN_NUM_TEST` | GPIO number for test pin | "25" |
| `CONFIG_EXAMPLE_PIN_NUM_INTR` | GPIO number for interrupt pin | "4" |

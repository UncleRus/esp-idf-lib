# Example for `max7219` driver

## Datasheet

[MAX7219/MAX7221](https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf)

## What it does

The example configures a seven-segment display (8 digits) with `max7219` device.

It shows some texts on the display in a loop.

## Wiring

The example assumes that you are using a seven-segment LED display module with
`MAX7219`. These modules usually come with:

* `MOSI` or `DIN` pin
* `CLK` pin
* `CS` pin
* `VCC` and `GND`

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_PIN_NUM_MOSI` | GPIO number for `MOSI`, or `DIN` | "13" for `esp8266`, "19" for `esp32c3`, `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_PIN_NUM_CLK`  | GPIO number for `CLK`  | "14" for `esp8266`, "18" for `esp32c3`, `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_PIN_CS`       | GPIO number for `CS` | "15 for `esp8266`, "15" for `esp32c3`, `esp32`, `esp32s2`, and `esp32s3` |

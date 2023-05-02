# Example for `ADS130E08` driver

The datasheet can be found [here](https://www.ti.com/lit/ds/symlink/ads130e08.pdf?ts=1666137782883).

## What it does

The example configures one `ADS130E08` device on a `SPI` bus, enabling it to be read via ISR or polling.

## Wiring

Connect `MOSI`, `MISO`, `SCLK`, `CS` and `INT` pins to the following pins:

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_MOSI_GPIO` | GPIO number for `MOSI` | "23" for `esp32` |
| `CONFIG_EXAMPLE_MISO_GPIO` | GPIO number for `MISO` | "19" for `esp32` |
| `CONFIG_EXAMPLE_SCLK_GPIO` | GPIO number for `SCLK` | "18" for `esp32` |
| `CONFIG_EXAMPLE_CS_GPIO` | GPIO number for `CS` | "5" for `esp32` |
| `CONFIG_EXAMPLE_INT_GPIO` | GPIO number for `INT` | "26" for `esp32` |

The driver was tested in ESP32 and ESP32S3, but should also work on other ESP boards.

## Notes

1. Please configure the SPI connections to ADS130E08 under `Example configuration` in `menuconfig`.
2. In order to read data via ISR, is necessary to enable SPI master and slave functions into IRAM under `Component config` -> `Driver configurations` -> `SPI configuration`.

## Output

```
I (1010) ads130e08: Fault status -> Positive 00, Negative f0
I (1010) ads130e08: Gpios level -> 0f
I (1010) ads130e08: Raw data -> CH1: -1 CH2: -1 CH3: -1 CH4: -1 CH5: -1 CH6: -1 CH7: -1 CH8: -1
```
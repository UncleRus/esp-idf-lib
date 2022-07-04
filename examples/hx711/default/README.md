# Example for `hx711` driver

## What it does

It shows averaged raw value from tensoresistor in a loop.

## Wiring

Connect `PD/SCK` and `DOUT` pins to the following GPIOs:

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_PD_SCK_GPIO` | GPIO number for `PD/SCK` pin | "4" for `esp8266` and `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_DOUT_GPIO` | GPIO number for `DOUT` pin | "5" for `esp8266` and `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

Set averaging samples under `Example configuration` in `menuconfig` ( `CONFIG_EXAMPLE_AVG_TIMES` ). The default is 5.

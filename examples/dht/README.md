# Example for `dht` driver

## What it does

It shows temperature and humidity in a loop.

## Wiring

Connect `DATA` pin to the following pin:

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_DATA_GPIO` | GPIO number for `DATA` pin | "4" for `esp8266` and `esp32c3`, "17" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

- Choose your chip type under `Example configuration` in `menuconfig`. The default is `DHT_TYPE_AM2301` (DHT21/DHT22/AM2301/AM2302/AM2321).
- You can enable internal pull-up resistor under `Example configuration` in `menuconfig`,
  but it's not recommended for stable operation.


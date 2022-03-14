# Example for `ultrasonic` component

## What it does

It measures and shows distance to an obstacle in loop.
This example is compatible with any ultrasonic module with `TRIGGER` and `ECHO` pins.

## Wiring

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_TRIGGER_GPIO` | GPIO number connected to `TRIGGER` pin | "4" for `esp8266` and `esp32c3`, "17" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_ECHO_GPIO` | GPIO number connected to `ECHO` pin | "5" for `esp8266` and `esp32c3`, "16" for `esp32`, `esp32s2`, and `esp32s3` |

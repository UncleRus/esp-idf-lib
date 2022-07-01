# Example for `mcp342x` driver

## Datasheet

[MCP3422/3/4](https://ww1.microchip.com/downloads/en/DeviceDoc/22088c.pdf)

## What it does

The example configures a `mcp342x` device (continuous mode,
`MCP342X_GAIN1`, `MCP342X_CHANNEL1`, and `MCP342X_RES_16`. see `main.c`).

It shows input voltage in a loop.

## Wiring

Connect an analog signal source to `CH1` of `MCP342X`.

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

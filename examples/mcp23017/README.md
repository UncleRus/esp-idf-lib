# Example for `mcp23x17` driver

## Datasheet

[MCP23017/MCP23S17](https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf)

## What it does

The example configures a `mcp23017` device. `PORTA0` is configured as input.
An interrupt handler is configured on `PORTA0`. `PORTB0` is configured as
output.

It toggles all 8 pins on output in a loop. When an interrupt is triggered, a
message is shown.

## Wiring

Connect all pins on `PORTB0` to LEDs and resistors. Connect digital input
sources to `PORTA0`.

Connect `CONFIG_EXAMPLE_INTA_GPIO` pin (the default is 19) to `INTA` pin of
`mcp23017`.

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

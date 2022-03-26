# Example for `mcp23x17` driver

## What it does

The example shows how to work with the MCP23S17 chip connected via SPI (see schematics below).

It sets up interrupt on pin PA0 of MCP23S17, triggered on both edges.
Then example blinks on PB0 in loop.

## Wiring

![Schematics](mp23s17_example.png?raw=true)

# Example for `led_effects` driver

## What it does

The example configures a matrix of `WS2812` LED
(`CONFIG_EXAMPLE_LED_MATRIX_WIDTH` x `CONFIG_EXAMPLE_LED_MATRIX_HEIGHT`,
`16x16` by default) at `CONFIG_EXAMPLE_LED_BRIGHTNESS` brightness (20 by
default).

It shows effects every `CONFIG_EXAMPLE_SWITCH_PERIOD_MS` (2000 millisecond by
default).

## Wiring

| Name | Description | Defaults |
|------|-------------|----------|
| `EXAMPLE_LED_GPIO` | GPIO number for `DIN`, the signal line for `WS2812` | "5" |

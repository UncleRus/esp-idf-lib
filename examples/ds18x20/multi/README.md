# Example for `ds18x20` driver

## What it does

The example configures one or `CONFIG_EXAMPLE_DS18X20_MAX_SENSORS` of
`ds18x20` devices (the default is 8) with an internal pullup on
`CONFIG_EXAMPLE_ONEWIRE_GPIO` (see below for defaults).  Use `menuconfig` to
change the default values under `Example configuration`.

It reads sensor values from the sensors and the address of each sensors, and
show them in a loop.

The internal (~47k) pull-ups of the ESP do appear to work, at least for simple
setups (one or two sensors connected with short leads), but do not technically
meet the pull-up requirements from the ds18x20 datasheet and may not always be
reliable.  For a real application, a proper 4.7k external pull-up resistor is
recommended instead!

## Wiring

Connect `DQ` pin to `CONFIG_EXAMPLE_ONEWIRE_GPIO`.

| Name | Description | Defaults                                                                          |
|------|-------------|-----------------------------------------------------------------------------------|
| `CONFIG_EXAMPLE_ONEWIRE_GPIO` | GPIO Number of 1-Wire bus, or `DQ` | "4" for `esp8266`, "18" for `esp32c3`, "17" for `esp32`, `esp32s2`, and `esp32s3` |

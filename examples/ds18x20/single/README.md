# Example for `ds18x20` driver

## What it does

The example configures a single `ds18x20` device with an internal pull-up on
`CONFIG_EXAMPLE_ONEWIRE_GPIO` (see below for defaults). You need to set
`CONFIG_EXAMPLE_DS18X20_ADDR` to your own sensors' address. Use `menuconfig`
to change the default values under `Example configuration`.

Use [`ds18x20_multi`](../ds18x20_multi) example to find out the sensor
address.

It reads sensor values from the sensor and show them in a loop.

The internal (~47k) pull-ups of the ESP do appear to work, at least for simple
setups (one or two sensors connected with short leads), but do not technically
meet the pull-up requirements from the ds18x20 datasheet and may not always be
reliable.  For a real application, a proper 4.7k external pull-up resistor is
recommended instead!

## Wiring

Connect `DQ` pin to `CONFIG_EXAMPLE_ONEWIRE_GPIO`.

| Name | Description | Defaults                                                                           |
|------|-------------|------------------------------------------------------------------------------------|
| `CONFIG_EXAMPLE_ONEWIRE_GPIO` | GPIO Number of 1-Wire bus, or `DQ` | "4" for `esp8266`, "18" for `esp32c3`, "17" for `esp32`, `esp32s2`, and `esp32s3` |

# Example for `TDA74xx` driver

The datasheet can be found at:

* [TDA7440](https://www.st.com/resource/en/datasheet/tda7440.pdf)
* [TDA7439DS](https://www.st.com/resource/en/datasheet/tda7439ds.pdf)

## What it does

The example configures a `TDA74xx`. It changes output volume in a loop.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

Additionally, you need audio input on a channel (`0` by default).

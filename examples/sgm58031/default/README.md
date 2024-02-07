# Example for `SGM58031` driver

The datasheet can be found [here](https://www.sg-micro.com/uploads/soft/20220609/1654744052.pdf).

## What it does

The example configures two `SMG58031` devices on an `I2C` bus. One has pin
`ADDR` connected to `GND`, and another has pin `ADDR` connected to `VCC`. The
example initializes two devices, and shows raw ADC values and voltages in a
loop.

## Wiring

Make the connections as below:

| S. No. | `SGM58031` | `ESP32`             |
|--------|-----------|---------------------|
| 1.     | `V_DD`    | `V_in` or 5V source |
| 2.     | `GND`     | `GND`               |
| 3.     | `SCL`     | See below           |
| 4.     | `SDA`     | See below           |
| 5.     | `A0`-`A3` | analog inputs       |

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

This example specifically demonstrates the simultaneous use of multiple
devices, which is why the default `CONFIG_EXAMPLE_DEV_COUNT` is 2. If you are
using only one IC then please change the value of `CONFIG_EXAMPLE_DEV_COUNT`
to 1 under `Example configuration` in `menuconfig`.

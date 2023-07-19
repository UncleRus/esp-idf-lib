# Single-Shot example for `SGM58031` driver

The datasheet can be found [here](https://www.sg-micro.com/uploads/soft/20220609/1654744052.pdf).

## What it does

The example configures a `SMG58031` device on an `I2C` bus with address selected
from config value `CONFIG_EXAMPLE_I2C_ADDRESS` (default to 0x48).
The example initializes the device and shows raw ADC values and voltages in a loop from each device input working in single-shot mode.

## Wiring

Make the connections as below:

| S. No. | `SGM58031` | `ESP32`            |
| ------ | ---------- | ------------------ |
| 1.     | `VDD`      | `Vin` or 5V source |
| 2.     | `GND`      | `GND`              |
| 3.     | `SCL`      | See below          |
| 4.     | `SDA`      | See below          |
| 5.     | `A0`-`A3`  | analog inputs      |

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name                            | Description           | Defaults                                                                         |
| ------------------------------- | --------------------- | -------------------------------------------------------------------------------- |
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

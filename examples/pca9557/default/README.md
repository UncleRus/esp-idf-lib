# Example for `pca9557` driver

## Datasheets

* [PCA9557](https://www.ti.com/lit/ds/symlink/pca9557.pdf)
* [PCA9537](https://www.nxp.com/docs/en/data-sheet/PCA9537.pdf)

## What it does

The example configures a `PCA9557` device, or a `PCA9537`. `IO0` is input, and
all other pins are output.

It blinks on `IO3`, and reads the signal level on `IO0`, and shows the level
in a loop.

## Wiring

Connect an LED and a resistor to `IO3` pin. Connect `IO0` to a digital signal
source.

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

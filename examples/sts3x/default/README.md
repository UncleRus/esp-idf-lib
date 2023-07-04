# Example for `sts3x` driver

## What it does

The example configures a `sts3x` device.

When `CONFIG_EXAMPLE_STS3X_DEMO_HL` is defined, the task that triggers a
measurement every 5 seconds. Due to power efficiency reasons it uses *single
shot* mode. In this example it uses the high level function *sts3x_measure()* to
perform one measurement in each cycle.

When `CONFIG_EXAMPLE_STS3X_DEMO_LL` is defined, the task that triggers a
measurement every 5 seconds. Due to power efficiency reasons it uses *single
shot* mode. In this example it starts the measurement, waits for the results
and fetches the results using separate functions.

Choose either `CONFIG_EXAMPLE_STS3X_DEMO_HL` or `CONFIG_EXAMPLE_STS3X_DEMO_LL`
in `make menuconfig` under `Example configuration`. The default is
`CONFIG_EXAMPLE_STS3X_DEMO_HL`.

## Wiring


Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

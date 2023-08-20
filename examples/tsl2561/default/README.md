# Example for `tsl2561` driver

## What it does

The example continually prints illuminance in lux to serial console. An
example:

```console
...
Found TSL2561 in package T/FN/CL
Lux: 1245
Lux: 1252
Lux: 1250
Lux: 1248
Lux: 1248
Lux: 1247
...
```

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

Choose I2C address under `Example configuration` in `menuconfig`. The default is
`TSL2561_I2C_ADDR_FLOAT` (0x39).

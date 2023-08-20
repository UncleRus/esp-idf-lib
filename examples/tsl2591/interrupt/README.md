# Example for `tsl2591` driver

# What the example does

The example continually prints illuminance in lux to serial console and reacts to
als and als no persistence interrupts.

## Wiring

Connect `SCL`, `SDA` and `INTR` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_INTR_GPIO` | GPIO number for `INTR` | "14" for `esp8266`, "7" for `esp32c3`, "5" for `esp32`, `esp32s2`, and `esp32s3` |

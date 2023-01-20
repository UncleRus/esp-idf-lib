# Wake-on-Motion (WoM) Example for `icm42670` driver

## What it does

This example configures the ICM42670 IMU to send an interrupt signal via a defined pin to the ESP if motion above a certain threshold is detected. This threshold (0.39*g by default) as well as the axes along movements are detected (all three axes by default) are configurable. Whether a WoM event was triggered or not is continuously checked in the main loop.

If the user moves the device, the "WoM event detected" state changes from false to true.

This Wake-on-Motion (WoM) feature can be used to wake the MCU from deep-sleep or perform any other operation if motion is detected. The WoM detection is performed in a low-power mode of the IMU.

## Wiring

Connect `SCL`, `SDA` and `INT` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "8" (`esp32c3`-based ESP-RS board), "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "10" (`esp32c3`-based ESP-RS board), "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_INT_INPUT_PIN` | GPIO number for `INT` | "0"|

## Notes

Choose I2C address under `Example configuration` in `menuconfig`. The default is
`ICM42670_I2C_ADDR_GND` (0x68).

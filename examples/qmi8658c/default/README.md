# Example for `qmi8658c` driver

## What it does

This example demonstrates basic usage of the QMI8658C 6-axis IMU sensor.  
It initializes the sensor, periodically reads accelerometer, gyroscope, and temperature data, and prints the results to the log.

## Wiring

Connect the `SCL` and `SDA` pins of the QMI8658C sensor to your ESP board with appropriate pull-up resistors.

| Name                            | Description           | Defaults (ESP32) |
|---------------------------------|-----------------------|------------------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for SCL   | 22               |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for SDA   | 21               |

## Usage

1. Build and flash the example to your ESP board.
2. Open the serial monitor to view sensor data output.

## Notes

- Make sure the I2C address matches your hardware (default: `0x6B`).
- Adjust the I2C pins in `sdkconfig` or directly in the example code if needed.
- The code under `main` follows ESP-IDF code style.
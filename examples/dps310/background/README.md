# Example for `dps310` driver (background mode)

## What it does

The example application initializes `DPS310` device. It enables the FIFO, and
set operation mode to background mode (one measurement per sec for temperature
and 2 measurements per sec for pressure).

In the loop, the sensor is polled by querying if FIFO is empty. Continue the
next loop if FIFO is empty, read one measurement from FIFO, otherwise.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

The default I2C address, which is used in this example, is `0x77`. Change the
address under `Example configuration` by `idf.py menuconfig`.

## Example output

```console
I (0) cpu_start: App cpu up.
I (226) cpu_start: Pro cpu start user code
I (226) cpu_start: cpu freq: 160000000
I (226) cpu_start: Application information:
I (230) cpu_start: Project name:     example-dps310-background
I (237) cpu_start: App version:      0.9.1-64-gd7ad7d9-dirty
I (243) cpu_start: Compile time:     Nov  3 2022 19:18:58
I (249) cpu_start: ELF file SHA256:  d6ee323571e3c6e3...
I (255) cpu_start: ESP-IDF:          v4.4.2-dirty
I (261) heap_init: Initializing. RAM available for dynamic allocation:
I (268) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (274) heap_init: At 3FFB2CE8 len 0002D318 (180 KiB): DRAM
I (280) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (286) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (293) heap_init: At 4008C9D0 len 00013630 (77 KiB): IRAM
I (300) spi_flash: detected chip: generic
I (304) spi_flash: flash io: dio
W (308) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (322) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (0) dps310_example_default: Initializing I2C
I (10) dps310_example_default: Initializing the device descriptor
I (10) dps310_example_default: Initializing the device
I (150) dps310_example_default: Waiting for the sensor to be ready for measurement
I (160) dps310_example_default: Flushing FIFO
I (160) dps310_example_default: Setting background measurement mode
I (160) dps310_example_default: Starting the loop
I (370) dps310_example_default: pressure: 101807.75 Pa
I (580) dps310_example_default: temperature: 27.93 °C
I (870) dps310_example_default: pressure: 101045.64 Pa
I (1380) dps310_example_default: pressure: 101045.03 Pa
I (1580) dps310_example_default: temperature: 27.93 °C
I (1880) dps310_example_default: pressure: 101043.59 Pa
I (2380) dps310_example_default: pressure: 101043.88 Pa
I (2590) dps310_example_default: temperature: 27.91 °C
I (2890) dps310_example_default: pressure: 101043.05 Pa
I (3390) dps310_example_default: pressure: 101043.48 Pa
I (3600) dps310_example_default: temperature: 27.91 °C
I (3890) dps310_example_default: pressure: 101042.98 Pa
I (4390) dps310_example_default: pressure: 101043.41 Pa
I (4600) dps310_example_default: temperature: 27.91 °C
I (4900) dps310_example_default: pressure: 101042.81 Pa
I (5400) dps310_example_default: pressure: 101043.41 Pa
I (5610) dps310_example_default: temperature: 27.91 °C
I (5900) dps310_example_default: pressure: 101042.90 Pa
I (6410) dps310_example_default: pressure: 101043.49 Pa
I (6610) dps310_example_default: temperature: 27.91 °C
I (6910) dps310_example_default: pressure: 101042.85 Pa
I (7410) dps310_example_default: pressure: 101043.41 Pa
I (7620) dps310_example_default: temperature: 27.90 °C
I (7910) dps310_example_default: pressure: 101042.59 Pa
I (8420) dps310_example_default: pressure: 101043.34 Pa
I (8630) dps310_example_default: temperature: 27.90 °C
...
```

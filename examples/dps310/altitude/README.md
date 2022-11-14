# Example for `dps310` driver (background mode)

## What it does

The example application initializes `DPS310` device, calibrates the sensor by
the real altitude.

It shows altitude in the loop.

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

Change `EXAMPLE_REAL_ALTITUDE` to the altitude of the device by `idf.py
menuconfig` (use a map, or Google Earth to find out).

## Example output

```console
I (0) cpu_start: App cpu up.
I (228) cpu_start: Pro cpu start user code
I (228) cpu_start: cpu freq: 160000000
I (228) cpu_start: Application information:
I (233) cpu_start: Project name:     example-dps310-background
I (239) cpu_start: App version:      0.9.1-73-g4d01ef1-dirty
I (246) cpu_start: Compile time:     Nov 13 2022 04:20:54
I (252) cpu_start: ELF file SHA256:  e0f948e27e257dd5...
I (258) cpu_start: ESP-IDF:          v4.4.2-dirty
I (263) heap_init: Initializing. RAM available for dynamic allocation:
I (270) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (277) heap_init: At 3FFB2CF0 len 0002D310 (180 KiB): DRAM
I (283) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (289) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (296) heap_init: At 4008C9D0 len 00013630 (77 KiB): IRAM
I (303) spi_flash: detected chip: generic
I (306) spi_flash: flash io: dio
W (310) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (325) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (0) dps310_example_default: Initializing I2C
I (10) dps310_example_default: Initializing the device descriptor
I (10) dps310_example_default: Initializing the device
I (140) dps310_example_default: Waiting for the sensor to be ready for measurement
I (1260) dps310: Calibration result:
I (1260) dps310: 	Pressure: 100799.67 (Pa)
I (1260) dps310: 	Calculated Pressure at sea level: 100991.00 (Pa)
I (1270) dps310: 	Calculated altitude: 15.99 (m)
I (1270) dps310: 	Real altitude: 16.00 (m)
I (1280) dps310: 	Offset: 0.01 (m)
I (1280) dps310_example_default: Setting background measurement mode
I (1290) dps310_example_default: Starting the loop
I (1340) dps310_example_default: altitude: 16.00 (m)
I (1390) dps310_example_default: altitude: 16.00 (m)
I (1440) dps310_example_default: altitude: 16.03 (m)
I (1490) dps310_example_default: altitude: 16.03 (m)
I (1540) dps310_example_default: altitude: 16.07 (m)
I (1590) dps310_example_default: altitude: 16.07 (m)
I (1640) dps310_example_default: altitude: 16.17 (m)
I (1690) dps310_example_default: altitude: 16.17 (m)
I (1740) dps310_example_default: altitude: 16.18 (m)
I (1790) dps310_example_default: altitude: 16.18 (m)
I (1840) dps310_example_default: altitude: 16.21 (m)
I (1890) dps310_example_default: altitude: 16.21 (m)
I (1940) dps310_example_default: altitude: 16.21 (m)
I (1990) dps310_example_default: altitude: 16.21 (m)
I (2040) dps310_example_default: altitude: 16.18 (m)
I (2090) dps310_example_default: altitude: 16.18 (m)
I (2140) dps310_example_default: altitude: 16.18 (m)
I (2190) dps310_example_default: altitude: 16.14 (m)
I (2240) dps310_example_default: altitude: 16.14 (m)
...
```

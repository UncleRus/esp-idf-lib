# Example for `tps63101x` driver

## What it does

Resets all registers to their default values and then reads all registers from IC.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "17" for `esp32`, `esp32s2`, and `esp32s3` |

## Example log

```console
I (0) cpu_start: App cpu up.
I (222) cpu_start: Pro cpu start user code
I (223) cpu_start: cpu freq: 160000000 Hz
I (223) cpu_start: Application information:
I (227) cpu_start: Project name:     example-tps63101x
I (233) cpu_start: App version:      0.9.4-133-gc6da185
I (239) cpu_start: Compile time:     May  9 2024 20:01:30
I (245) cpu_start: ELF file SHA256:  87fa1391764c6f1b...
I (251) cpu_start: ESP-IDF:          v5.1.2
I (256) cpu_start: Min chip rev:     v0.0
I (261) cpu_start: Max chip rev:     v3.99 
I (265) cpu_start: Chip rev:         v3.0
I (270) heap_init: Initializing. RAM available for dynamic allocation:
I (278) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (283) heap_init: At 3FFB2A50 len 0002D5B0 (181 KiB): DRAM
I (290) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (296) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (303) heap_init: At 4008D590 len 00012A70 (74 KiB): IRAM
I (310) spi_flash: detected chip: generic
I (313) spi_flash: flash io: dio
W (317) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (331) app_start: Starting scheduler on CPU0
I (336) app_start: Starting scheduler on CPU1
I (336) main_task: Started on CPU0
I (346) main_task: Calling app_main()
I (346) main_task: Returned from app_main()
I (346) Main: TPS63101X initialized.
I (2356) Main: Reset to defaults.
I (5356) Main: Reading control 1 register.
I (5356) Main: Result: 00000008, enable fast DVS: 1, enable SCP: 0, enable converter: 0
I (7356) Main: Reading vout register.
I (7356) Main: Result: 0000005c
I (9356) Main: Reading control 2 register.
I (9356) Main: Result: 00000045, force pwm: 0, fast ramp enable: 1, enable vout discharge: 0, cl ramp minimum: 0, ramp: 5
I (11366) Main: Finished.
```

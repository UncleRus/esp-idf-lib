# Example for `ds3231` driver

## What it does

It initializes the date and the time to 2016-10-09 13:50:10.

It shows date, time, and temperature in a loop.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

`CONFIG_NEWLIB_LIBRARY_LEVEL_NORMAL` must be `y` on `esp8266`.

## Log

```console
I (28) boot: ESP-IDF v4.0-dev-1446-g6b169d473 2nd stage bootloader
I (28) boot: compile time 19:12:04
I (29) boot: Enabling RNG early entropy source...
I (34) boot: SPI Speed      : 40MHz
I (39) boot: SPI Mode       : DIO
I (43) boot: SPI Flash Size : 2MB
I (47) boot: Partition Table:
I (50) boot: ## Label            Usage          Type ST Offset   Length
I (58) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (65) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (72) boot:  2 factory          factory app      00 00 00010000 00100000
I (80) boot: End of partition table
I (84) esp_image: segment 0: paddr=0x00010020 vaddr=0x3f400020 size=0x06150 ( 24912) map
I (102) esp_image: segment 1: paddr=0x00016178 vaddr=0x3ffb0000 size=0x02028 (  8232) load
I (106) esp_image: segment 2: paddr=0x000181a8 vaddr=0x40080000 size=0x00400 (  1024) load
I (111) esp_image: segment 3: paddr=0x000185b0 vaddr=0x40080400 size=0x07a60 ( 31328) load
I (133) esp_image: segment 4: paddr=0x00020018 vaddr=0x400d0018 size=0x0bbd0 ( 48080) map
I (150) esp_image: segment 5: paddr=0x0002bbf0 vaddr=0x40087e60 size=0x02f6c ( 12140) load
I (163) boot: Loaded app from partition at offset 0x10000
I (163) boot: Disabling RNG early entropy source...
I (163) cpu_start: Pro cpu up.
I (167) cpu_start: Application information:
I (172) cpu_start: Project name:     example-ds3231
I (177) cpu_start: App version:      2ae685b-dirty
I (183) cpu_start: Compile time:     Aug 23 2019 19:12:33
I (189) cpu_start: ELF file SHA256:  3ef2c5556f6ea7b8...
I (195) cpu_start: ESP-IDF:          v4.0-dev-1446-g6b169d473
I (201) cpu_start: Starting app cpu, entry point is 0x4008106c
I (0) cpu_start: App cpu up.
I (212) heap_init: Initializing. RAM available for dynamic allocation:
I (219) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (225) heap_init: At 3FFB3070 len 0002CF90 (179 KiB): DRAM
I (231) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (237) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (244) heap_init: At 4008ADCC len 00015234 (84 KiB): IRAM
I (250) cpu_start: Pro cpu start user code
I (268) spi_flash: detected chip: generic
I (268) spi_flash: flash io: dio
W (268) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (279) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
E (0) i2c: /usr/home/trombik/github/trombik/esp-idf/components/driver/i2c.c:353 (i2c_driver_delete):i2c driver install error
2016-10-09 13:50:10, 33.25 deg Cel
2016-10-09 13:50:10, 33.25 deg Cel
2016-10-09 13:50:10, 33.25 deg Cel
2016-10-09 13:50:10, 33.25 deg Cel
2016-10-09 13:50:11, 33.25 deg Cel
2016-10-09 13:50:11, 33.25 deg Cel
2016-10-09 13:50:11, 33.25 deg Cel
...
```

# Example for `si7021` driver

## What it does

When `CONFIG_EXAMPLE_CHIP_TYPE_SI70xx` is defined, it shows device model.

It shows temperature and humidity in a loop.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

Choose chip type under `Example configuration` in `menuconfig`. The default is
`SI70xx` family.

`CONFIG_NEWLIB_LIBRARY_LEVEL_NORMAL` must be `y` on `esp8266`.

## Example log

```console
ESP-ROM:esp32c3-api1-20210207
Build:Feb  7 2021
rst:0x1 (POWERON),boot:0xc (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fcd6100,len:0x1750
load:0x403ce000,len:0x930
load:0x403d0000,len:0x2d3c
entry 0x403ce000
I (30) boot: ESP-IDF v5.0-dev-1554-g20847eeb96 2nd stage bootloader
I (30) boot: compile time 16:09:02
I (30) boot: chip revision: 3
I (33) boot.esp32c3: SPI Speed      : 80MHz
I (38) boot.esp32c3: SPI Mode       : DIO
I (43) boot.esp32c3: SPI Flash Size : 4MB
I (48) boot: Enabling RNG early entropy source...
I (53) boot: Partition Table:
I (57) boot: ## Label            Usage          Type ST Offset   Length
I (64) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (71) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (79) boot:  2 factory          factory app      00 00 00010000 00100000
I (86) boot: End of partition table
I (91) esp_image: segment 0: paddr=00010020 vaddr=3c020020 size=07f80h ( 32640) map
I (104) esp_image: segment 1: paddr=00017fa8 vaddr=3fc8ba00 size=01518h (  5400) load
I (108) esp_image: segment 2: paddr=000194c8 vaddr=40380000 size=06b50h ( 27472) load
I (121) esp_image: segment 3: paddr=00020020 vaddr=42000020 size=1585ch ( 88156) map
I (138) esp_image: segment 4: paddr=00035884 vaddr=40386b50 size=04d7ch ( 19836) load
I (142) esp_image: segment 5: paddr=0003a608 vaddr=50000010 size=00010h (    16) load
I (147) boot: Loaded app from partition at offset 0x10000
I (150) boot: Disabling RNG early entropy source...
I (167) cpu_start: Pro cpu up.
I (176) cpu_start: Pro cpu start user code
I (176) cpu_start: cpu freq: 160000000 Hz
I (176) cpu_start: Application information:
I (179) cpu_start: Project name:     example-si7021
I (185) cpu_start: App version:      0.8.2-70-g6c7c888-dirty
I (191) cpu_start: Compile time:     Mar  1 2022 16:09:17
I (197) cpu_start: ELF file SHA256:  7e795c8a378045c0...
I (203) cpu_start: ESP-IDF:          v5.0-dev-1554-g20847eeb96
I (210) heap_init: Initializing. RAM available for dynamic allocation:
I (217) heap_init: At 3FC8DDB0 len 00032250 (200 KiB): DRAM
I (223) heap_init: At 3FCC0000 len 0001F060 (124 KiB): STACK/DRAM
I (230) heap_init: At 50000020 len 00001FE0 (7 KiB): RTCRAM
I (237) spi_flash: detected chip: generic
I (241) spi_flash: flash io: dio
I (245) sleep: Configure to isolate all GPIO pins in sleep state
I (252) sleep: Enable automatic switching of GPIO sleep configuration
I (259) cpu_start: Starting scheduler.
Temperature: 32.73
Humidity: 58.15
Temperature: 32.73
Humidity: 58.16
Temperature: 32.72
Humidity: 58.20
```

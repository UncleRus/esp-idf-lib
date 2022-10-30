# Example for `dps310` driver

## What it does

The example application initializes `DPS310` device. It reads temperature and
pressure in a loop.

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
ets Jun  8 2016 00:22:57

rst:0x1 (POWERON_RESET),boot:0x12 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2
load:0x3fff0030,len:6664
load:0x40078000,len:14848
load:0x40080400,len:3792
0x40080400: _init at ??:?

entry 0x40080694
I (27) boot: ESP-IDF v4.4.2-dirty 2nd stage bootloader
I (27) boot: compile time 18:31:58
I (27) boot: chip revision: 1
I (30) boot_comm: chip revision: 1, min. bootloader chip revision: 0
I (37) boot.esp32: SPI Speed      : 40MHz
I (42) boot.esp32: SPI Mode       : DIO
I (46) boot.esp32: SPI Flash Size : 2MB
I (51) boot: Enabling RNG early entropy source...
I (56) boot: Partition Table:
I (60) boot: ## Label            Usage          Type ST Offset   Length
I (67) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (75) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (82) boot:  2 factory          factory app      00 00 00010000 00100000
I (90) boot: End of partition table
I (94) boot_comm: chip revision: 1, min. application chip revision: 0
I (101) esp_image: segment 0: paddr=00010020 vaddr=3f400020 size=0a12ch ( 41260) map
I (124) esp_image: segment 1: paddr=0001a154 vaddr=3ffb0000 size=023a0h (  9120) load
I (128) esp_image: segment 2: paddr=0001c4fc vaddr=40080000 size=03b1ch ( 15132) load
I (137) esp_image: segment 3: paddr=00020020 vaddr=400d0020 size=18ac8h (101064) map
I (175) esp_image: segment 4: paddr=00038af0 vaddr=40083b1c size=08eb4h ( 36532) load
I (191) esp_image: segment 5: paddr=000419ac vaddr=50000000 size=00010h (    16) load
I (197) boot: Loaded app from partition at offset 0x10000
I (197) boot: Disabling RNG early entropy source...
I (211) cpu_start: Pro cpu up.
I (211) cpu_start: Starting app cpu, entry point is 0x40081108
0x40081108: call_start_cpu1 at /usr/home/trombik/github/trombik/esp-idf/components/esp_system/port/cpu_start.c:160

I (0) cpu_start: App cpu up.
I (225) cpu_start: Pro cpu start user code
I (225) cpu_start: cpu freq: 160000000
I (225) cpu_start: Application information:
I (230) cpu_start: Project name:     example-dps310-default
I (236) cpu_start: App version:      0.9.1-61-g6ff7b98
I (242) cpu_start: Compile time:     Oct 30 2022 18:32:16
I (248) cpu_start: ELF file SHA256:  6fd6da770ef72528...
I (254) cpu_start: ESP-IDF:          v4.4.2-dirty
I (259) heap_init: Initializing. RAM available for dynamic allocation:
I (266) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (272) heap_init: At 3FFB2CE8 len 0002D318 (180 KiB): DRAM
I (279) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (285) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (291) heap_init: At 4008C9D0 len 00013630 (77 KiB): IRAM
I (299) spi_flash: detected chip: generic
I (302) spi_flash: flash io: dio
W (306) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (320) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (0) dps310_example_default: Initializing I2C
I (10) dps310_example_default: Initializing the device descriptor
I (10) dps310_example_default: Initializing the device
I (80) dps310_example_default: Waiting for the sensor to be ready for measurement
I (90) dps310_example_default: Starting the loop
I (90) dps310_example_default: Setting manual temperature measurement mode
I (90) dps310_example_default: Waiting for the temperature value to be ready
I (310) dps310_example_default: Temperature: 31.32 °C
I (310) dps310_example_default: Setting manual pressure measurement mode
I (310) dps310_example_default: Waiting for the pressure value to be ready
I (530) dps310_example_default: Pressure: 100742.27 Pa
I (1530) dps310_example_default: Setting manual temperature measurement mode
I (1530) dps310_example_default: Waiting for the temperature value to be ready
I (1740) dps310_example_default: Temperature: 31.29 °C
I (1740) dps310_example_default: Setting manual pressure measurement mode
I (1740) dps310_example_default: Waiting for the pressure value to be ready
I (1960) dps310_example_default: Pressure: 100742.00 Pa
I (2960) dps310_example_default: Setting manual temperature measurement mode
I (2960) dps310_example_default: Waiting for the temperature value to be ready
I (3170) dps310_example_default: Temperature: 31.29 °C
...
```

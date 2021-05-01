# What the example does

The example continually prints illuminance in lux to serial console. An
example:

```console
I (33) boot: ESP-IDF v4.3-dev-2136-gb0150615d 2nd stage bootloader
I (33) boot: compile time 14:46:52
I (33) boot: chip revision: 1
I (37) boot_comm: chip revision: 1, min. bootloader chip revision: 0
I (52) boot.esp32: SPI Speed      : 40MHz
I (52) boot.esp32: SPI Mode       : DIO
I (53) boot.esp32: SPI Flash Size : 2MB
I (57) boot: Enabling RNG early entropy source...
I (63) boot: Partition Table:
I (66) boot: ## Label            Usage          Type ST Offset   Length
I (74) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (81) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (89) boot:  2 factory          factory app      00 00 00010000 00100000
I (96) boot: End of partition table
I (100) boot_comm: chip revision: 1, min. application chip revision: 0
I (107) esp_image: segment 0: paddr=00010020 vaddr=3f400020 size=0769ch ( 30364) map
I (128) esp_image: segment 1: paddr=000176c4 vaddr=3ffb0000 size=02894h ( 10388) load
I (132) esp_image: segment 2: paddr=00019f60 vaddr=40080000 size=00404h (  1028) load
I (136) esp_image: segment 3: paddr=0001a36c vaddr=40080404 size=05cach ( 23724) load
I (154) esp_image: segment 4: paddr=00020020 vaddr=400d0020 size=1642ch ( 91180) map
I (189) esp_image: segment 5: paddr=00036454 vaddr=400860b0 size=0601ch ( 24604) load
I (207) boot: Loaded app from partition at offset 0x10000
I (207) boot: Disabling RNG early entropy source...
I (218) cpu_start: Pro cpu up.
I (219) cpu_start: Starting app cpu, entry point is 0x40080fd8
0x40080fd8: call_start_cpu1 at /home/julian/.esp/esp-idf/components/esp_system/port/cpu_start.c:133

I (0) cpu_start: App cpu up.
I (233) cpu_start: Pro cpu start user code
I (233) cpu_start: cpu freq: 240000000
I (233) cpu_start: Application information:
I (237) cpu_start: Project name:     tsl2591-simple
I (243) cpu_start: App version:      0.7-15-ga476935-dirty
I (249) cpu_start: Compile time:     Dec 27 2020 14:46:50
I (255) cpu_start: ELF file SHA256:  8dd2ffc2f7790502...
I (261) cpu_start: ESP-IDF:          v4.3-dev-2136-gb0150615d
I (268) heap_init: Initializing. RAM available for dynamic allocation:
I (275) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (281) heap_init: At 3FFB30F8 len 0002CF08 (179 KiB): DRAM
I (287) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (293) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (300) heap_init: At 4008C0CC len 00013F34 (79 KiB): IRAM
I (307) spi_flash: detected chip: winbond
I (311) spi_flash: flash io: dio
W (315) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (329) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
Lux: 36.290527
Lux: 36.290527
Lux: 36.290527
...
```

## Connecting

| `TSL2591` | `ESP8266` | `ESP32` | `NodeMCU` |
|-----------|-----------|---------|-----------|
| `SCL`     | `GPIO5`   | `GPIO19`| `D1`      |
| `SDA`     | `GPIO4`   | `GPIO18`| `D2`      |

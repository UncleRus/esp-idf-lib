# What the example does

The example continually prints illuminance in lux to serial console and reacts to
als and als no persistence interrupts.

```console
I (33) boot: ESP-IDF v4.3-dev-2136-gb0150615d 2nd stage bootloader
I (33) boot: compile time 14:49:46
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
I (107) esp_image: segment 0: paddr=00010020 vaddr=3f400020 size=080bch ( 32956) map
I (129) esp_image: segment 1: paddr=000180e4 vaddr=3ffb0000 size=02894h ( 10388) load
I (133) esp_image: segment 2: paddr=0001a980 vaddr=40080000 size=00404h (  1028) load
I (136) esp_image: segment 3: paddr=0001ad8c vaddr=40080404 size=0528ch ( 21132) load
I (153) esp_image: segment 4: paddr=00020020 vaddr=400d0020 size=177e4h ( 96228) map
I (190) esp_image: segment 5: paddr=0003780c vaddr=40085690 size=06b54h ( 27476) load
I (209) boot: Loaded app from partition at offset 0x10000
I (209) boot: Disabling RNG early entropy source...
I (221) cpu_start: Pro cpu up.
I (221) cpu_start: Starting app cpu, entry point is 0x40080fe0
0x40080fe0: call_start_cpu1 at /home/julian/.esp/esp-idf/components/esp_system/port/cpu_start.c:133

I (0) cpu_start: App cpu up.
I (235) cpu_start: Pro cpu start user code
I (235) cpu_start: cpu freq: 240000000
I (235) cpu_start: Application information:
I (239) cpu_start: Project name:     tsl2561-interrupt
I (245) cpu_start: App version:      0.7-15-ga476935-dirty
I (251) cpu_start: Compile time:     Dec 27 2020 14:49:43
I (258) cpu_start: ELF file SHA256:  94af0c3cda458fdc...
I (264) cpu_start: ESP-IDF:          v4.3-dev-2136-gb0150615d
I (270) heap_init: Initializing. RAM available for dynamic allocation:
I (277) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (283) heap_init: At 3FFB30F8 len 0002CF08 (179 KiB): DRAM
I (289) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (296) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (302) heap_init: At 4008C1E4 len 00013E1C (79 KiB): IRAM
I (309) spi_flash: detected chip: winbond
I (313) spi_flash: flash io: dio
W (317) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (331) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (556) Main: Turn both interrupts on.
I (556) Main: Set interrupt thresholds.
I (556) Main: Set persistence filter.
I (556) Main: Set sleep after interrupt.
I (556) Main: Clear old interrupts.
I (566) Main: Start integration cycle.
I (1166) gpio: GPIO[4]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:2
I (1166) Main: Dispatch isr_task.
I (1166) Main: Dispatch tsl2591_task.
I (1166) Main: isr_task started.
I (1176) Main: Wait for interrupt.
I (1176) Main: tsl2591 task started.
Channel 0: 0x13
Channel 1: 0x6
Lux: 36.290527
GPIO 4: HIGH
Channel 0: 0x13
Channel 1: 0x6
Lux: 36.290527
GPIO 4: HIGH
Channel 0: 0x11c
Channel 1: 0x3c
Lux: 720.838318
...
```

## Connecting

| `TSL2591` | `ESP8266` | `ESP32` | `NodeMCU` |
|-----------|-----------|---------|-----------|
| `SCL`     | `GPIO5`   | `GPIO19`| `D1`      |
| `SDA`     | `GPIO4`   | `GPIO18`| `D2`      |
| `INT`     | `GPIO2`   | `GPIO2` | `D4`      |

# Example `LC709203F` driver

## What it does

Initialize `lc709203f` with parameters:
- Operational Mode
- APA (application adjustment package) 0x32. Value is selected to 2500mAh battery.
- Battery Profile 1 (4.2V max - 3.7 nominal).
- Temperature mode I2C.
    
Runs a loop that shows, every 10 seconds, temperature, voltage (V), rsoc (battery percentage) and ite(batttery percentage in 0.1% scale).

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

You can change `SCL` and `SDA` pins in Example configuration.

## Notes

Adafruit Feather esp32s3 is used to test. It requires to set GPIO7 HIGH to enable I2C pulls up. If you use any other board or configuration, remove from main:

```
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1 << GPIO_NUM_7);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_7, 1);  
```

## Log

```console
I (24) boot: ESP-IDF v4.4.1 2nd stage bootloader
I (25) boot: compile time 20:33:05
I (25) boot: chip revision: 0
I (26) boot.esp32s3: Boot SPI Speed : 80MHz
I (31) boot.esp32s3: SPI Mode       : DIO
I (36) boot.esp32s3: SPI Flash Size : 2MB
I (40) boot: Enabling RNG early entropy source...
I (46) boot: Partition Table:
I (49) boot: ## Label            Usage          Type ST Offset   Length
I (57) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (64) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (72) boot:  2 factory          factory app      00 00 00010000 00100000
I (79) boot: End of partition table
I (83) esp_image: segment 0: paddr=00010020 vaddr=3c020020 size=095d8h ( 38360) map
I (99) esp_image: segment 1: paddr=00019600 vaddr=3fc91d10 size=02720h ( 10016) load
I (102) esp_image: segment 2: paddr=0001bd28 vaddr=40374000 size=042f0h ( 17136) load
I (112) esp_image: segment 3: paddr=00020020 vaddr=42000020 size=1a8b4h (108724) map
I (137) esp_image: segment 4: paddr=0003a8dc vaddr=403782f0 size=09a14h ( 39444) load
I (146) esp_image: segment 5: paddr=000442f8 vaddr=50000000 size=00010h (    16) load
I (151) boot: Loaded app from partition at offset 0x10000
I (152) boot: Disabling RNG early entropy source...
I (166) cpu_start: Pro cpu up.
I (166) cpu_start: Starting app cpu, entry point is 0x403751cc
0x403751cc: call_start_cpu1 at F:/Tools/espressif/esp-idf/components/esp_system/port/cpu_start.c:160

I (0) cpu_start: App cpu up.
I (180) cpu_start: Pro cpu start user code
I (180) cpu_start: cpu freq: 160000000
I (180) cpu_start: Application information:
I (183) cpu_start: Project name:     default
I (188) cpu_start: App version:      0.8.3-172-gb2eda16-dirty
I (194) cpu_start: Compile time:     Jul 20 2022 20:32:22
I (200) cpu_start: ELF file SHA256:  a47e5b888ea0b844...
I (206) cpu_start: ESP-IDF:          v4.4.1
I (211) heap_init: Initializing. RAM available for dynamic allocation:
I (218) heap_init: At 3FC94E70 len 0004B190 (300 KiB): D/IRAM
I (225) heap_init: At 3FCE0000 len 0000EE34 (59 KiB): STACK/DRAM
I (231) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (237) heap_init: At 600FE000 len 00002000 (8 KiB): RTCRAM
I (244) spi_flash: detected chip: generic
I (249) spi_flash: flash io: dio
W (252) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (266) sleep: Configure to isolate all GPIO pins in sleep state
I (273) sleep: Enable automatic switching of GPIO sleep configuration
I (280) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (300) gpio: GPIO[7]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 1| Intr:0 
I (340) Exammple: Power Mode: 0x1
I (340) Exammple: APA: 0x32
I (340) Exammple: Battery Profile: 0x1
I (340) Exammple: Temp Mode: 0x0
I (350) Exammple: Temp: 20.0    Voltage: 3.75   RSOC: 20%       ITE: 20.4%
I (10350) Exammple: Temp: 20.0  Voltage: 3.94   RSOC: 20%       ITE: 20.5%
I (20350) Exammple: Temp: 20.0  Voltage: 3.99   RSOC: 20%       ITE: 20.6%
I (30350) Exammple: Temp: 20.0  Voltage: 3.94   RSOC: 20%       ITE: 20.8%
...
```


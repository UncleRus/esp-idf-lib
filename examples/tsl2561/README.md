# What the example does

The example continually prints illuminance in lux to serial console. An
example:

```console
I (43) boot: ESP-IDF v3.2-18-gf1b9e15b-dirty 2nd stage bootloader
I (43) boot: compile time 15:41:21
I (44) qio_mode: Enabling default flash chip QIO
I (52) boot: SPI Speed      : 40MHz
I (58) boot: SPI Mode       : QIO
I (64) boot: SPI Flash Size : 2MB
I (71) boot: Partition Table:
I (76) boot: ## Label            Usage          Type ST Offset   Length
I (87) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (99) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (110) boot:  2 factory          factory app      00 00 00010000 000f0000
I (122) boot: End of partition table
I (128) esp_image: segment 0: paddr=0x00010010 vaddr=0x40210010 size=0x338ac (211116) map
I (203) esp_image: segment 1: paddr=0x000438c4 vaddr=0x3ffe8000 size=0x0051c (  1308) load
I (204) esp_image: segment 2: paddr=0x00043de8 vaddr=0x3ffe851c size=0x0019c (   412) load
I (215) esp_image: segment 3: paddr=0x00043f8c vaddr=0x40100000 size=0x05adc ( 23260) load
I (234) boot: Loaded app from partition at offset 0x10000
I (281) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (291) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (481) phy_init: phy ver: 1055_12
I (491) reset_reason: RTC reset 2 wakeup 0 store 0, reason is 2
I (491) gpio: GPIO[4]| InputEn: 0| OutputEn: 1| OpenDrain: 1| Pullup: 0| Pulldown: 0| Intr:0
I (501) gpio: GPIO[5]| InputEn: 0| OutputEn: 1| OpenDrain: 1| Pullup: 0| Pulldown: 0| Intr:0
Found TSL2561 in package T/FN/CL
Lux: 1245
Lux: 1252
Lux: 1250
Lux: 1248
Lux: 1248
Lux: 1247
...
```

## Connecting

| `TSL2561` | `ESP8266` | `ESP32` | `NodeMCU` |
|-----------|-----------|---------|-----------|
| `SCL`     | `GPIO5`   | `GPIO19`| `D1`      |
| `SDA`     | `GPIO4`   | `GPIO18`| `D2`      |

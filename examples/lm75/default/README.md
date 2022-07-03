# Example for `lm75` driver

## What it does

The example application initializes LM75 device. In a loop, it reads
temperature value from the device, prints it to the console. During the loop,
changes the operation mode of the device.

When the device is in shutdown mode, the output should be same. When in normal
mode, the output should have a bit of deviation.

The default I2C address, which is used in this example, is 0x48. Some breakout
boards has A0, A1, and A2, for using different address. The address must be
between `LM75_I2C_ADDRESS_DEFAULT` (0x48) and `LM75_I2C_ADDRESS_MAX` (0x4f).

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "5" for `esp8266`, "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "4" for `esp8266`, "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |

## Notes

`CONFIG_NEWLIB_LIBRARY_LEVEL_NORMAL` must be `y` on `esp8266`.

## Example output

```console
I (71) boot: Partition Table:
I (76) boot: ## Label            Usage          Type ST Offset   Length
I (87) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (99) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (110) boot:  2 factory          factory app      00 00 00010000 000f0000
I (122) boot: End of partition table
I (128) esp_image: segment 0: paddr=0x00010010 vaddr=0x40210010 size=0x3e70c (255756) map
I (216) esp_image: segment 1: paddr=0x0004e724 vaddr=0x3ffe8000 size=0x0051c (  1308) load
I (217) esp_image: segment 2: paddr=0x0004ec48 vaddr=0x3ffe851c size=0x0019c (   412) load
I (228) esp_image: segment 3: paddr=0x0004edec vaddr=0x40100000 size=0x05d70 ( 23920) load
I (248) boot: Loaded app from partition at offset 0x10000
I (312) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (312) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (512) phy_init: phy ver: 1055_12
I (512) reset_reason: RTC reset 2 wakeup 0 store 0, reason is 2
I (522) lm75_example: Initializing I2C
I (522) lm75_example: Initializing LM75 descriptor
I (522) lm75_example: Initializing LM75
I (532) gpio: GPIO[4]| InputEn: 0| OutputEn: 1| OpenDrain: 1| Pullup: 0| Pulldown: 0| Intr:0
I (552) gpio: GPIO[5]| InputEn: 0| OutputEn: 1| OpenDrain: 1| Pullup: 0| Pulldown: 0| Intr:0
I (562) lm75_example: Wakeup LM75
I (572) lm75_example: Read default Overtemperature Shutdown temperature
I (582) lm75_example: OS temperature: 25.000000
I (592) lm75_example: Set Overtemperature Shutdown temperature to 25.000000
I (602) lm75_example: Read Overtemperature Shutdown temperature
I (612) lm75_example: Set OS polarlity to LM75_OS_ACTIVE_HIGH
I (622) lm75_example: Set OS polarlity to LM75_OS_ACTIVE_LOW
I (632) lm75_example: Starting the loop
Operation mode: shutdown
Temperature: 32.250
Temperature: 32.250
Temperature: 32.250
Temperature: 32.250
Temperature: 32.250
Temperature: 32.250
Temperature: 32.250
Temperature: 32.250
Temperature: 32.250
Temperature: 32.250
Operation mode: normal
Temperature: 32.250
Temperature: 32.250
Temperature: 32.125
Temperature: 32.125
Temperature: 32.000
Temperature: 32.125
Temperature: 32.000
Temperature: 32.000
Temperature: 32.125
Temperature: 32.250
Operation mode: shutdown
Temperature: 32.125
Temperature: 32.125
Temperature: 32.125
Temperature: 32.125
Temperature: 32.125
Temperature: 32.125
Temperature: 32.125
Temperature: 32.125
Temperature: 32.125
Temperature: 32.125
Operation mode: normal
Temperature: 32.000
Temperature: 32.125
Temperature: 32.125
Temperature: 32.000
...
```

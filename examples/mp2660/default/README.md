# Example for `mp2660` driver

## What it does

It reads all registers from IC

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "22" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "21" for `esp32`, `esp32s2`, and `esp32s3` |

## Example log

```console
I (0) cpu_start: App cpu up.
I (137) cpu_start: Pro cpu start user code
I (137) cpu_start: cpu freq: 80000000 Hz
I (137) cpu_start: Application information:
I (138) cpu_start: Project name:     Esp32.MP2660.I2C
I (143) cpu_start: App version:      0.0.1
I (147) cpu_start: Compile time:     Oct  9 2023 22:57:10
I (152) cpu_start: ELF file SHA256:  8231edda675415ce...
I (157) cpu_start: ESP-IDF:          v5.0.1
I (161) cpu_start: Min chip rev:     v0.0
I (165) cpu_start: Max chip rev:     v3.99
I (169) cpu_start: Chip rev:         v3.0
I (173) heap_init: Initializing. RAM available for dynamic allocation:
I (179) heap_init: At 3FFAE6E0 len 0000F480 (61 KiB): DRAM
I (184) heap_init: At 3FFC16B0 len 0001E950 (122 KiB): DRAM
I (189) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (195) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (200) heap_init: At 4008BB98 len 00014468 (81 KiB): IRAM
I (207) spi_flash: detected chip: generic
I (209) spi_flash: flash io: dio
I (214) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
E (00:00:03.084) i2cdev: Could not read from device [0x09 at 0]: 263 (ESP_ERR_TIMEOUT)
I (00:00:03.085) Main: Result reading fault register: ESP_ERR_TIMEOUT
E (00:00:03.088) i2cdev: Could not read from device [0x09 at 0]: -1 (ESP_FAIL)
I (00:00:03.095) Main: Result reading fault register: ESP_FAIL
I (00:00:03.101) Main: Result reading fault register: ESP_OK
I (00:00:03.106) Main: Result: 0000004f, EnHIZ: 0, VInMin3: 1, VInMin2: 0, VInMin1: 0, VInMin0: 1, IInLim_2: 1, IInLim_1: 1, IInLim_0: 1
I (00:00:03.118) Main: Result: 00000004, UVLO 0: 0, UVLO 1: 0, UVLO 2: 1, CEB: 0, WatchdogTimer: 0, RegisterReset: 0
I (00:00:03.128) Main: Result: 0000000e, ICC 0: 0, ICC 1: 1, ICC 2: 1, ICC 3: 1, ICC 4: 0
I (00:00:03.136) Main: Result: 0000004a, DSCHG 0: 0, DSCHG 1: 1, DSCHG 2: 0, DSCHG 3: 1, IPre 0: 0, IPre 1: 1
I (00:00:03.146) Main: Result: 000000a3, VBattReg 0: 1, VBattReg 1: 0, VBattReg 2: 0, VBattReg 3: 0, VBattReg 4: 1, VBattReg 5: 1, VBatt pre: 0, VBatt rech: 1
I (00:00:03.160) Main: Result: 0000004a, EnTerm: 1, WatchDog 0: 1, WatchDog 1: 0, EnTimer: 0, ChgTimer 0: 1, ChgTimer 1: 0, TermTmr: 0
I (00:00:03.172) Main: Result: 0000000b, TMR2XEn: 1, FetDis: 0, EnNtc: 0, TJReg 0: 0, TJReg 1: 0
I (00:00:03.180) Main: Result: 00000080, Rev 1: 0, Rev 0: 0, ChgStat 1: 0, ChgStat 0: 0, PPMStat: 0, PGStat: 0, ThermStat: 1
I (00:00:03.191) Main: Result: 00000080, WatchdogFault: 0, VinFault: 0, ThemSd: 0, BatFault 0: 0, StmrFault: 0
I (00:00:03.200) Main: Done.
```

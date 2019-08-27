## Makefile to build a test application

This Makefile injects `esp-idf-lib` components into `esp-idf`'s component path and
builds a unit test application of `ESP_IDF_LIB_TEST_CONPONENT_NAME`.

```
ets Jun N@!Í5ets Jun  8 2016 00:22:57

rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2
load:0x3fff0018,len:4
load:0x3fff001c,len:5420
load:0x40078000,len:10948
load:0x40080400,len:6312
entry 0x40080714
I (94) cpu_start: Pro cpu up.
I (94) cpu_start: Application information:
I (94) cpu_start: Project name:     unit-test-app
I (97) cpu_start: App version:      v4.0-dev-1446-g6b169d473-dirty
I (103) cpu_start: Compile time:     Aug 27 2019 16:37:49
I (110) cpu_start: ELF file SHA256:  fde49b40c6d48ad8...
I (116) cpu_start: ESP-IDF:          v4.0-dev-1446-g6b169d473-dirty
I (122) cpu_start: Starting app cpu, entry point is 0x40081034
I (0) cpu_start: App cpu up.
I (133) heap_init: Initializing. RAM available for dynamic allocation:
I (140) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (147) heap_init: At 3FFB3580 len 0002CA80 (178 KiB): DRAM
I (152) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (158) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (167) heap_init: At 4008A1E8 len 00015E18 (87 KiB): IRAM
I (171) cpu_start: Pro cpu start user code
I (189) efuse: Loading virtual efuse blocks from real efuses
I (190) spi_flash: detected chip: generic
I (190) spi_flash: flash io: dio
I (194) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.


Press ENTER to see the list of tests.


Here's the test menu, pick your combo:
(1)	"a silly test" [lm75]

Enter test for running.
Running a silly test...
/usr/home/trombik/github/trombik/esp-idf-lib/components/lm75/test/test_lm75.c:4:a silly test:PASS
Test ran in 25ms

-----------------------
1 Tests 0 Failures 0 Ignored
OK
Enter next test, or 'enter' to see menu
```

## Usage

Build the test application:

```
make build
```

Flash the application (make sure `ESPPORT` environment variable is defined):

```
make flash
```

Then, open serial console and type the number of test to execute. Note that
keyboard input is not echoed back.

Clean:

```
make clean
```

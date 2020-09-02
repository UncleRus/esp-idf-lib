# Example for `pir` driver

The example is based on the original work by [Nocluna](https://github.com/pantaluna),
which can be found at
[https://github.com/pantaluna/esp32_hcsr501_pir_sensor_using_lib](https://github.com/pantaluna/esp32_hcsr501_pir_sensor_using_lib).

## What it does

When the PIR module raises HIGH on DATA line, onboard LED turns on.

## How to test the example

Configure the example by the official method, either `make menuconfig` or
`idf.py menuconfig`. In the menu dialog, you can change GPIO number for DATA
line, GPIO number for onboard LED, etc.

Wire the MCU and the PIR module.

Build the example by the official method to build, either `make` or `idf.py`.

Flash the example by the official method to flash, either `make flash` or
`idf.py flash`.

Place the module so that the PIR module does NOT see human activity, i.e. your
body.

Power the MCU and the PIR module.

Wait for the module to calibrate the sensor on startup (60 sec).

Test the example. When the PID module detects human activity, one of onboard
LED will turns on. Wave your hand in front of the PIR sensor. The LED should
turn on. When the DATA line is LOW, the LED should turn off.

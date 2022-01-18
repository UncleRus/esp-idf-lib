# `led_strip_spi` example

## What the example does

The example runs the following effects in a loop.

- changes the colours of all the LEDs using [_color wheel_](https://duckduckgo.com/?q=color+wheel).
- simply rotates the colours (B -> G -> R)
- displays rainbow colors on the strip, scrolling the rainbow

## Configuration

You can change the number of pixels in a strip by `idf.py menuconfig` or `make
menuconfig`. See `CONFIG_EXAMPLE_N_PIXEL` under `Example configuration`. The
default is 8 pixels.

## Hookup

| Pin on `SK9822` | Destination                  |
|-----------------|------------------------------|
| `5V`            | `5V`                         |
| `CI`            | `GPIO14` (ESP32 and ESP8266) or `GPIO6` (ESP32C3) |
| `DI`            | `GPIO13` (ESP32 and ESP8266) or `GPIO7` (ESP32C3) |
| `GND`           | `GND`                        |

`SK9822` LED strip has `CI` and `DI` at both end of the strip. Make sure the
direction of data flow is correct. The `GPIO`s must be connected to the first
LED. My strip has arrows on the strip like below.

```text
--------------------- Data flow ------------------>

                  .--------------------------.
                  |                          |
                  |  .-------.    .-------.  |
GND    ----- GND -+--| --> G |----| --> G |--+- GND
GPIO14 ----- CI  -+--|       |----|       |--+- CI
GPIO13 ----- DI  -+--|  LED  |----|  LED  |--+- DI
5V/Vin ----- 5V  -+--|       |----|       |--+- 5V
                  |  `-------'    `-------'  |
                  | the first LED            |
                  `--------------------------'
```

## Notes

Start with a small number of LED pixels. Make sure the 5 V power source is
stiff enough. A `SK9822` draws up to 60 mA.  A cheap `ESP32` development board
with `AMS1117` can source 8 `WS2812` pixels from USB 5V `VIN` _with_ the
default example code.

Make data lines short. The clock speed is an order of `Mhz`.

`SK9822` is 5V device but you _might_ be able to drive it from ESP32, which is
3.3V device, without level shifter. See also [README.md for led_strip](../led_strip/README.md).

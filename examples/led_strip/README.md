# `led_strip` example

## What the example does

The example changes the colours of all the LEDs, white, blue, green, red, and
off at 1 Hz.

## Configuration

You can change the number of pixels in a strip by `menuconfig`. See
`CONFIG_LED_STRIP_LEN` under `Example configuration`. The default is 8 pixels.

Choose GPIO number in `menuconfig` to which `DI` (`DIN` on some products) is
connected. The default is `5`. `ESP32-C3-DevKitC-02` has an onboard WS2812
driven by GPIO8.

## Notes

Start with a small number of LED pixels. Make sure the 5 V power source is
stiff enough. A `WS2812` draws up to 60 mA.  A cheap `ESP32` development board
with `AMS1117` can source 8 `WS2812` pixels from USB 5V `VIN` _with_ the
default example code.

If your LED strip has many pixels:

- Do NOT use USB as a single power source. Provide different power source for
  the LED strip.
- Use 330 Ohm resistor on the data line, which prevents inrush current.
- Use a large capacitor on the power rail.

See [Adafruit NeoPixel Überguide](https://learn.adafruit.com/adafruit-neopixel-uberguide).

`WS2812` is 5V device, but it also supports lower voltage on the signal rails.
Make sure the distance between `GPIO` of the microcontroller and the first LED
is short. If you are lucky, you do not need voltage level converter. The
example runs fine with a single jumper wire on a half-size breadboard. Use
voltage level converter module, such as
[SparkFun Logic Level Converter](https://www.sparkfun.com/products/12009), or
level-shift IC, such as
[FXMA108](https://www.onsemi.com/products/standard-logic/level-translators/fxma108)
if it does not.

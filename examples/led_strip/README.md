# `led_strip` example

## What the example does

The example changes the colours of all the LEDs, white, blue, green, red, and
off at 1 Hz.

## Configuration

You can change the number of pixels in a strip by `make menuconfig`. See
`CONFIG_LED_STRIP_LEN` under `Example configuration`. The default is 8 pixels.

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

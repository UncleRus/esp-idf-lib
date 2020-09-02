# Generic driver for PIR motion detector modules

The software component "pir" was originally developed for the `HC-SR501` PIR
motion sensor module by Nocluna. The driver should work with most of generic
PIR modules.

The code was ported to `esp-idf-lib` so that it works on ESP8266. The original
code an be found at
[pantaluna/esp32_hcsr501_pir_sensor_using_lib](https://github.com/pantaluna/esp32_hcsr501_pir_sensor_using_lib).

The driver supports the following PIR motion sensor models:

- `HC-SR501` PIR motion sensor module with lens. This module is fully
  configurable without having to solder/unsolder tiny SMD resistors. This was
  the original product that became very popular.
- `MH-SR602` Micro PIR motion sensor module with lens (even smaller). The
  pin order is different (the pins on the PCB are labeled). This is currently
  the most popular PIR sensor and the data protocol is compatible with the
  `HC-SR501`. Quiescent current: 20uA.
- `AM312` Mini PIR motion sensor module with lens (smaller). The pin order
  is different (the pins on the PCB are labeled). This module is very
  sensitive to noise and not recommended for new projects.

However, only `HC-SR501` was tested while porting the driver.

## Example ESP-IDF project(s)

An example is available under [examples/pir](../../examples/pir) directory.

## Shop Product

- `HC-SR501` PIR motion sensor module with lens. Recommended if you want to
  adjust the module manually (with the potentiometers).
- `MH-SR602` Micro PIR motion sensor module with lens. Recommended but
  cannot be adjusted.
- `AM312` Mini PIR motion sensor module with lens. Not recommended because
  it is very sensitive to noise.

## Wiring Instructions

### Sensor PIN layout

```console
GND
VCC
DATA
```

### Sensor wiring up

Check the wiring scheme in the `doc` directory.

| DEVICE PIN  | MCU PIN                                |
|-------------|----------------------------------------|
| `GND`       | `GND`                                  |
| `VCC`       | `VCC 3.3V                              |
| `DATA`      | `GPIO#27` (Huzzah32 #27 bottomright-6) |

- Bypass capacitor: connect a 100nF XR7 ceramic capacitor between the sensor
  pins VCC and GND; as close as possible to the sensor. Check the wiring
  scheme in the [`doc`](doc) directory.
- CFB Low Pass Filter: connect a 100nF XR7 ceramic capacitor between the DATA
  pin of the sensor and GND. Connect a ferrite bead on the wire to the DATA
  pin of the sensor.

A 10K pullup resistor for the DATA pin is not needed because the pir software
component enables the ESP32's internal pullup resistor for that pin.

## Data Sheet

Go to the [`doc`](doc) directory for documents and photo's.

## Sensor Voltage Levels

- Operating Voltage: the specs say DC 4.5V-20V but the board also works
  with DC 3.3V (Ok for ESP32 boards).
- Voltage level on the DATA PIN: DC 3.3V (Ok for ESP32 boards).

## Data Pin Logic (no real protocol)

- DATA Pin logic: High 3.3V, Low 0V.
- The pin is HIGH (3.3V) when movement is detected (and stays so for X seconds
  during the time decay period). The time decay period is typically 2.5
  seconds.
- The pin is LOW (0V) when no movement is detected.

## Sensor FAQ

- Sensing angle: 110 degrees.
- Sensing distance: 0.3 - 5 meter.
- Some models can be extended with a LDR photoresistor model 5528
  (bright=8-20 KiloOhm, dark=1 MegaOhm) so that the module is only enabled
  when it is dark.
- The sensor is designed to adjust to slowly changing conditions that would
  happen normally as the day progresses and the environmental conditions
  change.
- The module contains the BISS0001 PIR motion detector IC. It processes the
  output of the analog sensor and transforms it in a digital signal.
- The HC-SR501 module contains the Holtek HT7133-1 regulator IC. It regulates
  the voltage 5V down to 3.3V.
- The sensor is only ready for use after 60 seconds after being powered-on.
  The sensor might output HIGH several times during that period. There should
  be as little motion as possible in the sensor's field of view during that
  period.
- The PIR (Passive Infra-Red) Sensor is a pyroelectric device that detects
  motion by measuring changes in the infrared levels emitted by surrounding
  objects. This motion can be detected by checking for a high signal on a
  single I/O pin. It measures heat levels so moving a wooden stick does not
  make it go ON.
- The sensor is sensitive to strong light sources (sunlight) and sudden wind
  flow (air outlets, airco's).



## Time Decay potentiometer (only the `HC-SR501` model)

- The pot on the left (see image).
- Defines how long the signal stays ON (and blocks further detections) after
  it has detected a movement.
- Recommended setting: minimum 3 seconds.
- Minimum = 3 seconds (fully left).
- Maximum = 300 seconds(fully right).
- @important When the time decay ends then the motion detector is disabled for
  3 seconds; the output signal will go Low.

## Distance Sensitivity potentiometer (only the `HC-SR501` model)

- The pot on the right (see image).
- Implies how far the sensor works. Minimum: 3 meters. Maximum: 8 meters.
- Recommended setting: 3 meters.
- Fully left: max range 7 meter, increases sensitivity.
- Fully right: max range 3 meter, decreases sensitivity.

## Jumper "Trigger Mode" (only the `HC-SR501` model)

- Recommended to use the Single trigger mode ("L").
- Modes:
  - Single Trigger Mode ("L"): output goes HIGH then LOW when triggered during
    the TIME DECAY period. Continuous motion results in repeated HIGH/LOW
    pulses. Output is LOW when idle.
  - Repeatable Trigger Mode ("H"): output remains HIGH when sensor is
    retriggered repeatedly during the TIME DECAY period. Output is LOW when
    idle.
- Jumper in the "L" position => uses Single Trigger Mode. Jumper-connect the
  two pins that are the closest to the corner of the board.

- Jumper in the "H" position => uses Repeatable trigger mode. Jumper-connect
  the two pins that are the furthest away from the corner the board.

## Battery power mods (only the HC-SR501 model)

Instructions @
[https://forum.mysensors.org/topic/1088/battery-powered-pir](https://forum.mysensors.org/topic/1088/battery-powered-pir).

## Reference: the ESP32 MJD Starter Kit SDK

Do you also want to create innovative IoT projects that use the ESP32 chip, or
ESP32-based modules, of the popular company Espressif? Well, I did and still
do. And I hope you do too.

The objective of this well documented Starter Kit is to accelerate the
development of your IoT projects for ESP32 hardware using the ESP-IDF
framework from Espressif and get inspired what kind of apps you can build for
ESP32 using various hardware modules.

Go to
[https://github.com/pantaluna/esp32-mjd-starter-kit](https://github.com/pantaluna/esp32-mjd-starter-kit).

# Example for `wiegand` reader component

## What it does

In this example, a wiegand reader is configured and as soon as data comes from it, it's displayed using a callback. 

## Wiring

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_D0_GPIO` | GPIO number for `D0` line | "4" for `esp8266` and `esp32c3`, "16" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_D1_GPIO` | GPIO number for `D1` line | "5" for `esp8266` and `esp32c3`, "17" for `esp32`, `esp32s2`, and `esp32s3` |

The voltage level used for wigand protocol is usually +5V, but can be higher. The inputs of the Espressif microcontrollers
are 5V tolerant, but for safety reasons, use opto-isolation or a simple level shifter:

![Level shifter](wiegand_connection.png?raw=true)

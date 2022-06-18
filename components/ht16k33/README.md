# HT16K33 driver

Driver for Holtek HT16K33 LED Controller.

## Usage

See the datasheet at https://www.holtek.com/documents/10179/116711/HT16K33v120.pdf .
Chip RAM is 16-byte memory where each bit represents idividual pixel.
It is possbile to change I2C address using jumpers, see the datasheet.

*  Init descriptor with `ht16k33_init_desc()`.
*  Init device with `ht16k33_init()`.
*  Optionally, set up brightness using `ht16k33_set_brightness()`.
*  Turn display on and set blinking mode using `ht16k33_display_setup()`. Unfortunately, its a single command in HT16K33.
*  Write RAM state to set individual pixels using `ht16k33_ram_write()`.
*  At the end, deinitialize using `ht16k33_free_desc()`.

See the example at `examples/ht16k33`.


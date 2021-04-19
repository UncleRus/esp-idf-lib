.. _led_strip_spi:

led_strip_spi - SPI-based driver for SK9822/APA102
==================================================

Supported LEDs
--------------

The driver supports the following LEDs.

- `SK9822`
- `APA102`

.. warning:: The driver should work with APA102, but not tested.

Supported targets
-----------------

The driver supports the following targets.

- `ESP32`-family, including `ESP32S2`
- `ESP8266`

Because the SPI driver in `esp-idf` differs from the one in `ESP8266 RTOS
SDK`, the members of `led_strip_spi_t` &mdash; a descriptor to configure and
manage LED strip &mdash; are different.

.. note:: If you have better ideas to unify the two structures, I would love to hear.

Usage
-----

Call :cpp:func:`led_strip_spi_install()`.

.. code-block:: C

   #include <led_strip_spi.h>

   ESP_ERROR_CHECK(led_strip_spi_install());

Create :cpp:type:`led_strip_spi_t` and initialize it with :c:macro:`LED_STRIP_SPI_DEFAULT()`.

.. code-block:: C

   led_strip_spi_t strip = LED_STRIP_SPI_DEFAULT();

Configure the LED strip descriptor. First, set `length`.

.. code-block:: C

   strip.length = N_PIXEL;

:cpp:type:`led_strip_spi_t` is an alias of either :cpp:struct:`led_strip_spi_esp32_t` or
:cpp:struct:`led_strip_spi_esp8266_t`.

For ESP32, provide `spi_device_handle_t`, and set `max_transfer_sz` to the
maximum size of the SPI data to transfer. :c:macro:`LED_STRIP_SPI_BUFFER_SIZE()` macro is
provided to calculate the size from the number of pixels. Optionally, set SPI
bus clock speed.

.. code-block:: C

   #if HELPER_TARGET_IS_ESP32
   spi_device_handle_t device_handle;
   strip.device_handle = device_handle;
   strip.max_transfer_sz = LED_STRIP_SPI_BUFFER_SIZE(N_PIXEL);
   strip.clock_speed_hz = 1000000 * 10;
   #endif

Then, call :cpp:func:`led_strip_spi_init()`.

.. code-block:: C

   led_strip_spi_init(&strip);

The strip is now ready. Use :cpp:func:`led_strip_spi_set_pixel()` and other functions to
modify the buffer. The buffer is sent to the bus when calling
:c:func:`led_strip_spi_flush()`. See the example provided.

SPI signals and GPIO pins
-------------------------

The driver uses hardware SPI to drive LED strip. While ESP32-family chips
can route SPI signals to GPIOs using GPIO matrix, ESP8266 cannot.

SPI clock speed
---------------

On ESP32, dedicated `IO_MUX` pins can clock faster (80Mhz) than GPIO pins
through GPIO matrix (40Mhz), but the maximum clock frequency of `SK9822` is
30Mhz, which is below the maximum clock frequency of GPIO matrix. As such,
`IO_MUX` has no practical benefits here.

Known issues
------------

`SK9822` has bits to control global brightness, but the driver does not
support it yet.

.. doxygengroup:: led_strip_spi_esp32
   :members:

.. doxygengroup:: led_strip_spi_esp8266
   :members:

.. doxygengroup:: led_strip_spi
   :members:

.. doxygengroup:: led_strip_spi_sk9822
   :members:

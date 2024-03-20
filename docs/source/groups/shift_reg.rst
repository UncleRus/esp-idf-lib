``Shift Register``
==================

This driver can be used as an interface with a shift register in (still
in progress) and out (such as
`74HC595 <https://www.ti.com/lit/ds/symlink/sn74hc595.pdf>`__).

Usage
-----

To use the library, it needs to be configured before initialize. To do
so a ``shift_reg_config_t`` needs to be set.

Configuration
~~~~~~~~~~~~~

The ``shift_reg_config_t`` struct store all necessaries configurtions
and value of aech register.

-  ``uint8_t num_reg`` - Number of registers which will be used
-  ``uint8_t *reg_value`` - Vector for the last value of all registers;
   it can be used for know what is the actual value of the registers

Mode configuration
~~~~~~~~~~~~~~~~~~

-  ``struct mode``

   -  ``shift_reg_dir_t dir`` - Direction mode of the shift register
   -  ``shift_reg_bit_mode_t bit_mode`` - Bit mode

Pin configurations
~~~~~~~~~~~~~~~~~~

-  ``struct pin``

   -  ``gpio_num_t clk`` - Clock pin
   -  ``gpio_num_t data`` - Data/Signal pin
   -  ``gpio_num_t latch`` - Latch pin

Direcition mode
^^^^^^^^^^^^^^^

The ``shift_reg_dir_t`` says the direction of the register.

-  ``SHIFT_DIR_OUTPUT`` - Set the register as output
-  ``SHIFT_DIR_INPUT`` - Set the register as input
-  ``SHIFT_DIR_INPUT_OUTPUT`` - Set the register as output and input

First bit configuration
^^^^^^^^^^^^^^^^^^^^^^^

The ``shift_reg_bit_mode_t`` says the bit mode of the data.

-  ``SHIFT_BIT_MODE_LSB`` - Start send data from the lowest significant
   bit
-  ``SHIFT_BIT_MODE_MSB`` - Start send data from the most significandt
   bit

Initialization
~~~~~~~~~~~~~~

To initialize the shift register it is going to need to call the
``esp_err_t shift_reg_init(shift_reg_config_t *dev)``, by passing the
created shift register configurations.

De-initialization
~~~~~~~~~~~~~~~~~

Since the ``uint8_t *reg_value`` is created accordingly of the number of
registers, the vector allocate the necessary size in the heap memory, so
``esp_err_t shift_reg_deinit(shift_reg_config_t *dev)`` can be used to
free this memory.

Sending the data
~~~~~~~~~~~~~~~~

To send the intended data, call the
``esp_err_t shift_reg_send(uint8_t *data, uint8_t len, shift_reg_config_t *dev)``
function, where:

-  ``data`` - the address of the bening of the data
-  ``len`` - the length of the data in bytes
-  ``dev`` - the shift register interface to be used

**NOTE**: The data sent will be just in the internal memory
(register(s)) of the shift register(s), and not reflected in the pins.
See lataching pins to see how the pins can be set.

Latching pins
~~~~~~~~~~~~~

To set the pins as it is in the internal memory of the register(s), the
function ``esp_err_t shift_reg_latch(shift_reg_config_t *dev);`` needs
to be called. It will be latched by the passed shifter interface.

TODO
----

-  ☐ Implement input shift register
-  ☐ Interface with `hd44780 <../hd44780/>`__

Author
======

Jaime Albuquerque

-  GitHub: `jaimealbq <https://github.com/jaimealbq>`__
-  GitLab: `jaimealbq <https://gitlab.com/jaimealbq>`__
-  LinkedIn: `Jaime
   Albuquerque <https://www.linkedin.com/in/jaimealbq>`__

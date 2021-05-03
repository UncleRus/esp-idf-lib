.. _led_strip:

led_strip - RMT-based driver for WS2812B/SK6812/APA106 LED strips
=================================================================

.. warning:: This component does not support ESP8266 RTOS SDK!

.. warning:: If you try to use this driver simultaneously with Wi-Fi, you may
   encounter RMT transmission bugs. To avoid them, simply initialize device
   from the task bound to the second processor core.
   
   Interrupt handlers assigned during the initialization of the RMT driver are
   bound to the core on which the initialization took place. 
   
.. doxygengroup:: led_strip
   :members:


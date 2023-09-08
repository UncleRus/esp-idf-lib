# RMT-based ESP-IDF driver for WS2812B/SK6812/APA106 LED strips

**WARNING!** If you try to use this driver simultaneously with Wi-Fi, you may
encounter RMT transmission bugs. To avoid them, simply initialize device
descriptor from the task bound to the second processor core.

Interrupt handlers assigned during the initialization of the RMT driver are
bound to the core on which the initialization took place.

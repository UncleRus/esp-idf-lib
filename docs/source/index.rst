.. esp-idf-lib Documentation master file.

##########################
ESP-IDF Components library
##########################

Components for Espressif ESP32 `ESP-IDF framework <https://github.com/espressif/esp-idf>`_.

Some of them ported from `esp-open-rtos <https://github.com/SuperHouse/esp-open-rtos>`_.

**Project home**:

- GitHub: https://github.com/UncleRus/esp-idf-lib
- GitLab: https://gitlab.com/UncleRus/esp-idf-lib

==========
How to use
==========

ESP32 ESP-IDF
=============

Clone respository:

.. code-block:: shell
   
   cd ~/my/work/path
   git clone git@github.com:UncleRus/esp-idf-lib.git

or 

.. code-block:: shell
   
   cd ~/my/work/path
   git clone git@gitlab.com:UncleRus/esp-idf-lib.git

Add path to components in your project CMakeLists.txt:

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.5)
   set(EXTRA_COMPONENT_DIRS $ENV{HOME}/my/work/path/esp-idf-lib/components)
   include($ENV{IDF_PATH}/tools/cmake/project.cmake)
   project(my-esp-project)
   
ESP8266 RTOS SDK
================

Clone respository:

.. code-block:: shell
   
   cd ~/my/work/path
   git clone git@github.com:UncleRus/esp-idf-lib.git

or 

.. code-block:: shell
   
   cd ~/my/work/path
   git clone git@gitlab.com:UncleRus/esp-idf-lib.git

Add path to components in your project makefile, e.g:

.. code-block:: make
   
   PROJECT_NAME := my-esp-project
   EXTRA_COMPONENT_DIRS := $(HOME)/my/work/path/esp-idf-lib/components
   EXCLUDE_COMPONENTS := max7219 mcp23x17 led_strip max31865 ls7366r max31855
   include $(IDF_PATH)/make/project.mk

====================
Available components
====================

Common libraries
================
.. toctree::
   :maxdepth: 1

   groups/i2cdev
   groups/onewire
   groups/lib8tion
   groups/color
   groups/noise
   groups/framebuffer
   groups/calibration

Real-time clocks
================
.. toctree::
   :maxdepth: 1

   groups/ds1302
   groups/ds1307
   groups/ds3231
   groups/pcf8563

Humidity & temperature sensors
==============================
.. toctree::
   :maxdepth: 1

   groups/dht
   groups/sht3x
   groups/sht4x
   groups/si7021
   groups/ds18x20
   groups/max31725
   groups/lm75
   groups/bme680
   groups/mcp9808
   groups/mcp960x
   groups/tsys01
   groups/aht
   groups/hts221
   groups/max31865
   groups/bh1900nux
   groups/hdc1000
   groups/max31855
   groups/sts21
   groups/sts3x
   groups/am2320

Pressure sensors
================
.. toctree::
   :maxdepth: 1

   groups/bmp180
   groups/bmp280
   groups/bme680
   groups/dps310
   groups/ms5611

Air quality/Gas sensors
=======================
.. toctree::
   :maxdepth: 1

   groups/sgp40
   groups/ccs811
   groups/mhz19b
   groups/scd4x
   groups/scd30
   groups/sfa3x

ADC/DAC
=======
.. toctree::
   :maxdepth: 1
   
   groups/ads111x
   groups/hx711
   groups/pcf8591
   groups/mcp4725
   groups/mcp342x
   groups/ads130e08
   groups/sgm58031

Power/Current monitors
======================
.. toctree::
   :maxdepth: 1
   
   groups/ina219
   groups/ina260
   groups/ina3221

Magnetic sensors
================
.. toctree::
   :maxdepth: 1

   groups/hmc5883l
   groups/qmc5883l
   groups/lsm303

Light sensors
=============
.. toctree::
   :maxdepth: 1

   groups/bh1750
   groups/tsl2561
   groups/tsl4531
   groups/tsl2591
   groups/veml7700

GPIO expanders
==============
.. toctree::
   :maxdepth: 1

   groups/pcf8574
   groups/pcf8575
   groups/tca95x5
   groups/mcp23008
   groups/mcp23x17
   groups/pca9557
   groups/tca6424a

LED drivers
===========
.. toctree::
   :maxdepth: 1

   groups/led_strip
   groups/led_strip_spi
   groups/ht16k33
   groups/max7219

Input controls
==============
.. toctree::
   :maxdepth: 1

   groups/button
   groups/encoder
   groups/ls7366r

Inertial measurement units
==========================
.. toctree::
   :maxdepth: 1

   groups/icm42670
   groups/mpu6050
   groups/l3gx
   groups/lsm303

Battery controllers
===================
.. toctree::
   :maxdepth: 1

   groups/lc709203f
   groups/max1704x
   groups/mp2660

Other
=====
.. toctree::
   :maxdepth: 1

   groups/hd44780
   groups/pca9685
   groups/ultrasonic
   groups/tda74xx
   groups/rda5807m
   groups/tca9548
   groups/ds3502
   groups/wiegand


===========
Information
===========
.. toctree::
   :maxdepth: 1
   
   chips
   

==================
Indices and tables
==================

* :ref:`genindex`
* :ref:`search`

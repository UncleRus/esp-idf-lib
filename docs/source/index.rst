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

Add path to components in your project makefile, e.g:

.. code-block:: make
   
   PROJECT_NAME := my-esp-project
   EXTRA_COMPONENT_DIRS := $(HOME)/my/work/path/esp-idf-lib/components
   include $(IDF_PATH)/make/project.mk
   
or in CMakeLists.txt:

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
   EXCLUDE_COMPONENTS := max7219 mcp23x17
   include $(IDF_PATH)/make/project.mk

====================
Available components
====================

Common drivers
==============
.. toctree::
   :maxdepth: 1

   groups/i2cdev
   groups/onewire

Real-time clocks
================
.. toctree::
   :maxdepth: 1

   groups/ds1302
   groups/ds1307
   groups/ds3231

Humidity & temperature sensors
==============================
.. toctree::
   :maxdepth: 1

   groups/dht
   groups/sht3x
   groups/si7021
   groups/ds18x20
   groups/max31725
   groups/lm75
   groups/bme680
   
Pressure sensors
================
.. toctree::
   :maxdepth: 1

   groups/bmp180
   groups/bmp280
   groups/bme680
   groups/ms5611

ADC/DAC
=======
.. toctree::
   :maxdepth: 1
   
   groups/ads111x
   groups/hx711
   groups/pcf8591
   groups/mcp4725

Power/Current monitors
======================
.. toctree::
   :maxdepth: 1
   
   groups/ina219
   groups/ina3221

Magnetic sensors
================
.. toctree::
   :maxdepth: 1

   groups/hmc5883l
   groups/qmc5883l
   
Light sensors
=============
.. toctree::
   :maxdepth: 1

   groups/bh1750
   groups/tsl2561
   groups/tsl4531

GPIO expanders
==============
.. toctree::
   :maxdepth: 1

   groups/pcf8574
   groups/pcf8575
   groups/tca95x5
   groups/mcp23008
   groups/mcp23x17

Other
=====
.. toctree::
   :maxdepth: 1

   groups/hd44780
   groups/max7219
   groups/pca9685
   groups/ultrasonic
   groups/encoder
   groups/tda74xx
   

==================
Indices and tables
==================

* :ref:`genindex`
* :ref:`search`

# ESP-IDF Components library

[![Build Status](https://github.com/UncleRus/esp-idf-lib/workflows/Build%20examples/badge.svg)](https://github.com/UncleRus/esp-idf-lib/actions?query=workflow%3A%22Build+examples%22)
[![Build the documentation](https://github.com/UncleRus/esp-idf-lib/workflows/Build%20the%20documentation/badge.svg)](https://github.com/UncleRus/esp-idf-lib/actions?query=workflow%3A%22Build+the+documentation%22)
[![Docs Status](https://readthedocs.org/projects/esp-idf-lib/badge/?version=latest&style=flat)](https://esp-idf-lib.readthedocs.io/en/latest/)

Components for Espressif ESP32 [ESP-IDF framework](https://github.com/espressif/esp-idf)
and [ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK).

Part of them ported from [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos).

## Supported versions of frameworks and devices

| Chip           | Framework          | Versions
|----------------|--------------------|-----------------------
| ESP32          | ESP-IDF            | master, v4.x, v.3.3.5
| ESP32-S2 *[1]* | ESP-IDF            | master, v4.x, v.3.3.5
| ESP8266  *[2]* | ESP8266 RTOS SDK   | master, v3.3

[1] *Use "`idf.py set-target esp32s2`" before "`idf.py menuconfig`" to change
the chip type.*

[2] *Due to the incompatibility of ESP8266 drivers and hardware, some
libraries are not* *supported on ESP8266 (see "ESP8266" column in the tables).*

## How to use

### ESP32

Clone this repository somewhere, e.g.:

```Shell
cd ~/myprojects/esp
git clone https://github.com/UncleRus/esp-idf-lib.git
```

Add path to components in your [project makefile](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system-legacy.html),
e.g:

```Makefile
PROJECT_NAME := my-esp-project
EXTRA_COMPONENT_DIRS := /home/user/myprojects/esp/esp-idf-lib/components
include $(IDF_PATH)/make/project.mk
```

or in [CMakeLists.txt](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html):

```CMake
cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS /home/user/myprojects/esp/esp-idf-lib/components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my-esp-project)
```

or with CMake [FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html)

```CMake
cmake_minimum_required(VERSION 3.11)
include(FetchContent)
FetchContent_Declare(
  espidflib
  GIT_REPOSITORY https://github.com/UncleRus/esp-idf-lib.git
)
FetchContent_MakeAvailable(espidflib)
set(EXTRA_COMPONENT_DIRS ${espidflib_SOURCE_DIR}/components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my-esp-project)
```

### ESP8266 RTOS SDK

Clone this repository somewhere, e.g.:

```Shell
cd ~/myprojects/esp
git clone https://github.com/UncleRus/esp-idf-lib.git
```

Add path to components in your [project makefile](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/api-guides/build-system.html),
e.g:

```Makefile
PROJECT_NAME := my-esp-project
EXTRA_COMPONENT_DIRS := /home/user/myprojects/esp/esp-idf-lib/components
EXCLUDE_COMPONENTS := max7219 mcp23x17 led_strip
include $(IDF_PATH)/make/project.mk
```

See [GitHub examples](https://github.com/UncleRus/esp-idf-lib/tree/master/examples)
or [GitLab examples](https://gitlab.com/UncleRus/esp-idf-lib/tree/master/examples).

## Documentation

- [Documentation](https://esp-idf-lib.readthedocs.io/en/latest/)
- [Frequently asked questions](FAQ.md)

## Components

### Common libraries

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **i2cdev**     | I2C utilites                                                            | MIT     | Yes     | Yes
| **onewire**    | Bit-banging 1-Wire driver                                               | MIT *   | Yes     | *No*
| **lib8tion**   | Math functions specifically designed for LED programming                | MIT     | Yes     | -
| **color**      | Common library for RGB and HSV colors                                   | MIT     | Yes     | -
| **noise**      | Noise generation functions                                              | MIT     | Yes     | -
| **framebuffer** | RGB framebuffer component                                              | MIT     | Yes     | -

### Real-time clocks

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **ds1302**     | Driver for DS1302 RTC module                                            | BSD     | Yes     | *No*
| **ds1307**     | Driver for DS1307 RTC module                                            | BSD     | Yes     | Yes
| **ds3231**     | Driver for DS1337 RTC and DS3231 high precision RTC module              | MIT     | Yes     | Yes
| **pcf8563**    | Driver for PCF8563 real-time clock/calendar                             | BSD     | Yes     | Yes

### Humidity & temperature sensors

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **dht**        | Driver for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), Itead Si7021   | BSD     | Yes     | *No*
| **sht3x**      | Driver for Sensirion SHT3x digital temperature and humidity sensor      | BSD     | Yes     | Yes
| **sht4x**      | Driver for Sensirion SHT4x digital temperature and humidity sensor      | BSD     | Yes     | Yes
| **si7021**     | Driver for Si7013/Si7020/Si7021/HTU21D/SHT2x and compatible             | BSD     | Yes     | Yes
| **ds18x20**    | Driver for DS18B20/DS18S20 families of 1-Wire temperature sensor ICs    | BSD     | Yes     | *No*
| **max31725**   | Driver for MAX31725/MAX31726 temperature sensors                        | BSD     | Yes     | Yes
| **lm75**       | Driver for LM75, a digital temperature sensor and thermal watchdog      | ISC     | Yes     | Yes
| **mcp9808**    | Driver for MCP9808, precision digital temperature sensor                | BSD     | Yes     | Yes
| **mcp960x**    | Driver for MCP9600/MCP9601, thermocouple EMF to temperature converter   | BSD     | Yes     | Yes
| **tsys01**     | Driver for precision digital temperature sensor TSYS01                  | BSD     | Yes     | Yes

### Pressure sensors

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **bmp180**     | Driver for BMP180 digital pressure sensor                               | MIT     | Yes     | Yes
| **bmp280**     | Driver for BMP280/BME280 digital pressure sensor                        | MIT     | Yes     | Yes
| **bme680**     | Driver for BME680 digital environmental sensor                          | BSD     | Yes     | Yes
| **ms5611**     | Driver for barometic pressure sensor MS5611-01BA03                      | BSD     | Yes     | Yes

### Air quality/Gas sensors

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **sgp40**      | SGP40 Indoor Air Quality Sensor for VOC Measurements                    | BSD     | Yes     | Yes
| **ccs811**     | Driver for AMS CCS811 digital gas sensor                                | BSD     | Yes     | Yes
| **mhz19b**     | Driver for MH-Z19B NDIR CO2 sensor                                      | BSD     | Yes     | *No*

### ADC/DAC

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **ads111x**    | Driver for ADS1113/ADS1114/ADS1115 and ADS1013/ADS1014/ADS1015 I2C ADC  | BSD     | Yes     | Yes
| **hx711**      | Driver for HX711 24-bit ADC for weigh scales                            | BSD     | Yes     | *No*
| **mcp4725**    | Driver for 12-bit DAC MCP4725                                           | BSD     | Yes     | Yes
| **pcf8591**    | Driver for 8-bit ADC and an 8-bit DAC PCF8591                           | BSD     | Yes     | Yes
| **mcp342x**    | Driver for 18-Bit, delta-sigma ADC MCP3426/MCP3427/MCP3428              | BSD     | Yes     | Yes

### Power/Current monitors

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **ina3221**    | Driver for INA3221 shunt and bus voltage monitor                        | MIT     | Yes     | Yes
| **ina219**     | Driver for INA219/INA220 bidirectional current/power monitor            | BSD     | Yes     | Yes
| **ina260**     | Driver for INA260 precision digital current and power monitor           | BSD     | Yes     | Yes

### Magnetic sensors

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **hmc5883l**   | Driver for HMC5883L 3-axis digital compass                              | BSD     | Yes     | Yes
| **qmc5883l**   | Driver for QMC5883L 3-axis magnetic sensor                              | BSD     | Yes     | Yes

### Light sensors

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **bh1750**     | Driver for BH1750 light sensor                                          | BSD     | Yes     | Yes
| **tsl2561**    | Driver for light-to-digital converter TSL2561                           | BSD     | Yes     | Yes
| **tsl4531**    | Driver for digital ambient light sensor TSL4531                         | BSD     | Yes     | Yes
| **tsl2591**    | Driver for light-to-digital converter TSL2591                           | MIT     | Yes     | Yes

### GPIO expanders

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **pcf8574**    | Driver for PCF8574 remote 8-bit I/O expander for I2C-bus                | MIT     | Yes     | Yes
| **pcf8575**    | Driver for PCF8575 remote 16-bit I/O expander for I2C-bus               | MIT     | Yes     | Yes
| **tca95x5**    | Driver for TCA9535/TCA9555 remote 16-bit I/O expanders for I2C-bus      | BSD     | Yes     | Yes
| **mcp23008**   | Driver for 8-bit I2C GPIO expander MCP23008                             | BSD     | Yes     | Yes
| **mcp23x17**   | Driver for I2C/SPI 16 bit GPIO expanders MCP23017/MCP23S17              | BSD     | *No*    | Yes

### Addressable LEDs

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **led_strip**  | RMT-based driver for WS2812B/SK6812/APA106 LED strips                   | MIT     | *No*    | Yes
| **led_strip_spi** | SPI-based driver for SK9822/APA102 LED strips                        | MIT     | Yes     | Yes

### Input controls

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **button**     | HW timer-based driver for GPIO buttons                                  | BSD     | Yes     | Yes
| **encoder**    | HW timer-based driver for incremental rotary encoders                   | BSD     | Yes     | Yes

### Other

| Component      | Description                                                             | License | ESP8266 | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------|---------------
| **ultrasonic** | Driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05              | BSD     | Yes     | *No*
| **hd44780**    | Universal driver for HD44780 LCD display                                | BSD     | Yes     | *No*
| **pca9685**    | Driver for 16-channel, 12-bit PWM PCA9685                               | BSD     | Yes     | Yes
| **max7219**    | Driver for 8-Digit LED display drivers, MAX7219/MAX7221                 | BSD     | *No*    | Yes
| **tda74xx**    | Driver for TDA7439/TDA7439DS/TDA7440D audioprocessors                   | MIT     | Yes     | Yes
| **tca9548**    | Driver for TCA9548A/PCA9548A low-voltage 8-channel I2C switch           | BSD     | Yes     | Yes
| **rda5807m**   | Driver for single-chip broadcast FM radio tuner RDA5807M                | BSD     | Yes     | Yes
| **ds3502**     | Driver for nonvolatile digital potentiometer DS3502                     | MIT     | Yes     | Yes
| **wiegand**    | Wiegand protocol receiver                                               | BSD     | Yes     | *No*

## Library maintainers

- [Ruslan V. Uss](https://github.com/UncleRus)
- [Tomoyuki Sakurai](https://github.com/trombik)

## Credits

- [Tomoyuki Sakurai](https://github.com/trombik), developer of the LM75 and
  SK9822/APA102 drivers, author of the RTOS SDK ESP82666 support, master CI
- [Gunar Schorcht](https://github.com/gschorcht), developer of SHT3x, BME680
  and CCS811 drivers
- [Brian Schwind](https://github.com/bschwind), developer of TS2561 and
  TSL4531 drivers
- [Andrej Krutak](https://github.com/andree182), developer of BH1750 driver
- Frank Bargstedt, developer of BMP180 driver
- [sheinz](https://github.com/sheinz), developer of BMP280 driver
- [Jonathan Hartsuiker](https://github.com/jsuiker), developer of DHT driver
- [Grzegorz Hetman](https://github.com/hetii), developer of DS18B20 driver
- [Alex Stewart](https://github.com/astewart-consensus), developer of DS18B20 driver
- [Richard A Burton](mailto:richardaburton@gmail.com), developer of DS3231 driver
- [Bhuvanchandra DV](https://github.com/bhuvanchandra), developer of DS3231 driver
- [Zaltora](https://github.com/Zaltora), developer of INA3231 driver
- [Bernhard Guillon](https://gitlab.com/mrnice), developer of MS5611-01BA03 driver
- [Pham Ngoc Thanh](https://github.com/panoti), developer of PCF8591 driver
- [Lucio Tarantino](https://github.com/dianlight), developer of ADS111x driver
- [Julian Dörner](https://github.com/juliandoerner), developer of TSL2591 driver
- [FastLED community](https://github.com/FastLED), developers of `lib8tion`,
  `color` and `noise` libraries
- [Erriez](https://github.com/Erriez), developer of MH-Z19B driver
- [David Douard](https://github.com/douardda), developer of MH-Z19B driver

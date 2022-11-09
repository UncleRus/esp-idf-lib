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
| ESP32          | ESP-IDF            | All officially supported versions (see [Support Period Policy](https://github.com/espressif/esp-idf/blob/master/SUPPORT_POLICY.md)) and `master`
| ESP32-S2 *[1]* | ESP-IDF            | All officially supported versions and `master`
| ESP32-C3 *[1]* | ESP-IDF            | All officially supported versions and `master`
| ESP8266  *[2]* | ESP8266 RTOS SDK   | `master`, v3.4

[1] *Use "`idf.py set-target esp32s2`" or "`idf.py set-target esp32c3`" before "`idf.py menuconfig`" to change
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
EXCLUDE_COMPONENTS := max7219 mcp23x17 led_strip max31865 ls7366r max31855
include $(IDF_PATH)/make/project.mk
```

See [GitHub examples](https://github.com/UncleRus/esp-idf-lib/tree/master/examples)
or [GitLab examples](https://gitlab.com/UncleRus/esp-idf-lib/tree/master/examples).

## Documentation

- [Documentation](https://esp-idf-lib.readthedocs.io/en/latest/)
- [Frequently asked questions](FAQ.md)

## Components

### ADC/DAC libraries

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **ads111x**              | Driver for ADS1113/ADS1114/ADS1115 and ADS1013/ADS1014/ADS1015 I2C ADC           | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ads130e08**            | Driver for ADS130E08 ADC                                                         | MIT     | `esp32`            | Yes
| **hx711**                | Driver for HX711 24-bit ADC for weigh scales                                     | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **mcp342x**              | Driver for 18-Bit, delta-sigma ADC MCP3426/MCP3427/MCP3428                       | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **mcp4725**              | Driver for 12-bit DAC MCP4725                                                    | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **pcf8591**              | Driver for 8-bit ADC and an 8-bit DAC PCF8591                                    | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Air quality sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **ccs811**               | Driver for AMS CCS811 digital gas sensor                                         | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **mhz19b**               | Driver for MH-Z19B NDIR CO₂ sensor                                               | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **scd30**                | Driver for SCD30 CO₂ sensor                                                      | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **scd4x**                | Driver for SCD40/SCD41 miniature CO₂ sensor                                      | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **sgp40**                | Driver for SGP40 Indoor Air Quality Sensor for VOC Measurements                  | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Common libraries

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **color**                | Common library for RGB and HSV colors                                            | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **esp_idf_lib_helpers**  | Common support library for esp-idf-lib                                           | ISC     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **framebuffer**          | RGB framebuffer component                                                        | MIT     | `esp32`, `esp32s2`, `esp32c3` | Yes
| **i2cdev**               | ESP-IDF I2C master thread-safe utilities                                         | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **lib8tion**             | Math functions specifically designed for LED programming                         | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **noise**                | Noise generation functions                                                       | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **onewire**              | Bit-banging 1-Wire driver                                                        | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No

### Current and power sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **ina219**               | Driver for INA219/INA220 bidirectional current/power monitor                     | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ina260**               | Driver for INA260 precision digital current and power monitor                    | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ina3221**              | Driver for INA3221 shunt and bus voltage monitor                                 | ISC     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### GPIO expanders

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **mcp23008**             | Driver for 8-bit I2C GPIO expander MCP23008                                      | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **mcp23x17**             | Driver for I2C/SPI 16 bit GPIO expanders MCP23017/MCP23S17                       | BSD-3   | `esp32`, `esp32s2`, `esp32c3` | Yes
| **pca9557**              | Driver for PCA9537/PCA9557/TCA9534 remote 4/8-bit I/O expanders for I2C-bus      | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **pcf8574**              | Driver for PCF8574 remote 8-bit I/O expander for I2C-bus                         | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **pcf8575**              | Driver for PCF8575 remote 16-bit I/O expander for I2C-bus                        | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **tca95x5**              | Driver for TCA9535/TCA9555 remote 16-bit I/O expanders for I2C-bus               | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Gas sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **ccs811**               | Driver for AMS CCS811 digital gas sensor                                         | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **mhz19b**               | Driver for MH-Z19B NDIR CO₂ sensor                                               | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **scd30**                | Driver for SCD30 CO₂ sensor                                                      | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **scd4x**                | Driver for SCD40/SCD41 miniature CO₂ sensor                                      | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Humidity sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **aht**                  | Driver for AHT10/AHT15/AHT20 temperature and humidity sensor                     | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **bme680**               | Driver for BME680 digital environmental sensor                                   | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **dht**                  | Driver for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), Itead Si7021            | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **hdc1000**              | Driver for HDC1000 temperature and humidity sensor                               | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **hts221**               | Driver for HTS221 temperature and humidity sensor.                               | ISC     | `esp32`, `esp32s2`, `esp32c3` | Yes
| **sht3x**                | Driver for Sensirion SHT30/SHT31/SHT35 digital temperature and humidity sensor   | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **sht4x**                | Driver for Sensirion SHT40/SHT41/SHT45 digital temperature and humidity sensor   | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **si7021**               | Driver for Si7013/Si7020/Si7021/HTU2xD/SHT2x and compatible temperature and humidity sensors | BSD-3   | `esp32`, `esp32c3`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Input device drivers

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **button**               | HW timer-based driver for GPIO buttons                                           | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **encoder**              | HW timer-based driver for incremental rotary encoders                            | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ls7366r**              | Driver for LS7366R Quadrature Encoder Counter                                    | MIT     | `esp32`, `esp32s2`, `esp32c3` | Yes

### LED drivers

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **ht16k33**              | HT16K33 LED controller driver                                                    | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **led_strip**            | RMT-based driver for WS2812B/SK6812/APA106/SM16703 LED strips                    | MIT     | `esp32`, `esp32s2`, `esp32c3` | Yes
| **led_strip_spi**        | SPI-based driver for SK9822/APA102 LED strips                                    | MIT     | `esp32`, `esp32c3`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **max7219**              | Driver for 8-Digit LED display drivers, MAX7219/MAX7221                          | BSD-3   | `esp32`, `esp32s2`, `esp32c3` | Yes

### Light sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **bh1750**               | Driver for BH1750 light sensor                                                   | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **tsl2561**              | Driver for light-to-digital converter TSL2561                                    | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **tsl2591**              | Driver for light-to-digital converter TSL2591                                    | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **tsl4531**              | Driver for digital ambient light sensor TSL4531                                  | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Magnetic sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **hmc5883l**             | Driver for 3-axis digital compass HMC5883L and HMC5983L                          | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **qmc5883l**             | Driver for QMC5883L 3-axis magnetic sensor                                       | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Other misc libraries

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **ds3502**               | Driver for nonvolatile digital potentiometer DS3502                              | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **example**              | An example component                                                             | ISC     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **hd44780**              | Driver for HD44780 compatible LCD text displays                                  | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **lc709203f**            | Driver for LC709203F battery fuel gauge                                          | ISC     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **pca9685**              | Driver for 16-channel, 12-bit PWM PCA9685                                        | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **rda5807m**             | Driver for single-chip broadcast FM radio tuner RDA5807M                         | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **tca9548**              | Driver for TCA9548A/PCA9548A low-voltage 8-channel I2C switch                    | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **tda74xx**              | Driver for TDA7439/TDA7439DS/TDA7440D audioprocessors                            | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ultrasonic**           | Driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05                       | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **wiegand**              | Wiegand protocol receiver                                                        | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No

### Pressure sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **bme680**               | Driver for BME680 digital environmental sensor                                   | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **bmp180**               | Driver for BMP180 digital pressure sensor                                        | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **bmp280**               | Driver for BMP280/BME280 digital pressure sensor                                 | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **dps310**               | Driver for DPS310 barometric pressure sensor                                     | ISC     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ms5611**               | Driver for barometic pressure sensor MS5611-01BA03                               | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Real-time clocks

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **ds1302**               | Driver for DS1302 RTC module                                                     | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **ds1307**               | Driver for DS1307 RTC module                                                     | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ds3231**               | Driver for DS1337 RTC and DS3231 high precision RTC module                       | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **pcf8563**              | Driver for PCF8563 real-time clock/calendar                                      | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

### Temperature sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|--------------
| **aht**                  | Driver for AHT10/AHT15/AHT20 temperature and humidity sensor                     | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **bh1900nux**            | Driver for BH1900NUX temperature sensor                                          | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **bme680**               | Driver for BME680 digital environmental sensor                                   | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **bmp180**               | Driver for BMP180 digital pressure sensor                                        | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **bmp280**               | Driver for BMP280/BME280 digital pressure sensor                                 | MIT     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **dht**                  | Driver for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), Itead Si7021            | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **dps310**               | Driver for DPS310 barometric pressure sensor                                     | ISC     | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ds18x20**              | Driver for DS18B20/DS18S20 families of 1-Wire temperature sensor ICs             | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | No
| **hdc1000**              | Driver for HDC1000 temperature and humidity sensor                               | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **hts221**               | Driver for HTS221 temperature and humidity sensor.                               | ISC     | `esp32`, `esp32s2`, `esp32c3` | Yes
| **lm75**                 | Driver for LM75, a digital temperature sensor and thermal watchdog               | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **max31725**             | Driver for MAX31725/MAX31726 temperature sensors                                 | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **max31855**             | Driver for MAX31855 cold-junction compensated thermocouple-to-digital converter  | BSD-3   | `esp32`, `esp32s2`, `esp32c3` | Yes
| **max31865**             | Driver for MAX31865 resistance converter for platinum RTDs                       | BSD-3   | `esp32`, `esp32s2`, `esp32c3` | Yes
| **mcp960x**              | Driver for MCP9600/MCP9601, thermocouple EMF to temperature converter            | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **mcp9808**              | Driver for MCP9808 Digital Temperature Sensor                                    | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **ms5611**               | Driver for barometic pressure sensor MS5611-01BA03                               | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **sht3x**                | Driver for Sensirion SHT30/SHT31/SHT35 digital temperature and humidity sensor   | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **sht4x**                | Driver for Sensirion SHT40/SHT41/SHT45 digital temperature and humidity sensor   | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **si7021**               | Driver for Si7013/Si7020/Si7021/HTU2xD/SHT2x and compatible temperature and humidity sensors | BSD-3   | `esp32`, `esp32c3`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **sts21**                | Driver for STS21 temperature sensor                                              | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes
| **tsys01**               | Driver for precision digital temperature sensor TSYS01                           | BSD-3   | `esp32`, `esp8266`, `esp32s2`, `esp32c3` | Yes

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
- [Nate Usher](https://github.com/nated0g), developer of SCD30 driver
- [Josh Kallus](https://github.com/Jkallus), developer of LS7366R driver
- [saasaa](https://github.com/saasaa), developer of HTS221 driver
- [Timofei Korostelev](https://github.com/chudsaviet), developer of HT16K33 driver
- [Jose Manuel Perez](https://github.com/jmpmscorp), developer of LC709203F driver
- [Weslley Duarte](https://github.com/weslleymfd), developer of ADS130E08 driver

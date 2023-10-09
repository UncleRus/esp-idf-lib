# ESP-IDF Components library

[![Main CI process](https://github.com/UncleRus/esp-idf-lib/actions/workflows/ci.yml/badge.svg)](https://github.com/UncleRus/esp-idf-lib/actions/workflows/ci.yml)
[![Docs Status](https://readthedocs.org/projects/esp-idf-lib/badge/?version=latest&style=flat)](https://esp-idf-lib.readthedocs.io/en/latest/)

Components for Espressif ESP32 [ESP-IDF framework](https://github.com/espressif/esp-idf)
and [ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK).

Part of them ported from [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos).

## Supported versions of frameworks and devices

| Chip     | Framework        | Versions                                                                                                                                         |
|----------|------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| ESP32-xx | ESP-IDF          | All officially supported versions (see [Support Period Policy](https://github.com/espressif/esp-idf/blob/master/SUPPORT_POLICY.md)) and `master` |
| ESP8266  | ESP8266 RTOS SDK | `master`, v3.4                                                                                                                                   |

*See "Supported on" column for each of the components.*

## How to use

### ESP32-xx

Clone this repository somewhere, e.g.:

```Shell
cd ~/myprojects/esp
git clone https://github.com/UncleRus/esp-idf-lib.git
```

Add path to components in your [CMakeLists.txt](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html):
e.g:

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

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **ads111x**              | Driver for ADS1113/ADS1114/ADS1115 and ADS1013/ADS1014/ADS1015 I2C ADC           | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ads130e08**            | Driver for ADS130E08 ADC                                                         | MIT     | esp32, esp32s3     | yes           |
| **hx711**                | Driver for HX711 24-bit ADC for weigh scales                                     | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **mcp342x**              | Driver for 18-Bit, delta-sigma ADC MCP3426/MCP3427/MCP3428                       | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **mcp4725**              | Driver for 12-bit DAC MCP4725                                                    | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **pcf8591**              | Driver for 8-bit ADC and an 8-bit DAC PCF8591                                    | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sgm58031**             | Driver for SGM58031 16-bit I2C ADC                                               | ISC     | esp32, esp8266, esp32s2, esp32s3, esp32c3 | yes           |


### Air quality sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **ccs811**               | Driver for AMS CCS811 digital gas sensor                                         | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **mhz19b**               | Driver for MH-Z19B NDIR CO₂ sensor                                               | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **scd30**                | Driver for SCD30 CO₂ sensor                                                      | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **scd4x**                | Driver for SCD40/SCD41 miniature CO₂ sensor                                      | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sfa3x**                | Driver for SFA30 formaldehyde detection module (I2C)                             | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sgp40**                | Driver for SGP40 Indoor Air Quality Sensor for VOC Measurements                  | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |


### Battery controllers

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **lc709203f**            | Driver for LC709203F battery fuel gauge                                          | ISC     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **max1704x**             | Driver for MAX17043/MAX17044/MAX17048/MAX17049 battery fuel gauge                | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **mp2660**               | Driver for MP2660 5V USB, 500mA, I2C-Controlled Linear Charger with Power Path Management for Single-Cell Li-Ion Battery | BSD-3-Clause | esp32, esp32s2, esp32c3 | yes           |


### Common libraries

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **calibration**          | Multi-point calibration library                                                  | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | n/a           |
| **color**                | Common library for RGB and HSV colors                                            | MIT     | esp32, esp8266, esp32s2, esp32c3 | n/a           |
| **esp_idf_lib_helpers**  | Common support library for esp-idf-lib                                           | ISC     | esp32, esp8266, esp32s2, esp32c3 | n/a           |
| **framebuffer**          | RGB framebuffer component                                                        | MIT     | esp32, esp32s2, esp32c3 | n/a           |
| **i2cdev**               | ESP-IDF I2C master thread-safe utilities                                         | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **lib8tion**             | Math functions specifically designed for LED programming                         | MIT     | esp32, esp8266, esp32s2, esp32c3 | n/a           |
| **noise**                | Noise generation functions                                                       | MIT     | esp32, esp8266, esp32s2, esp32c3 | n/a           |
| **onewire**              | Bit-banging 1-Wire driver                                                        | MIT     | esp32, esp8266, esp32s2, esp32c3 | no            |


### Current and power sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **ina219**               | Driver for INA219/INA220 bidirectional current/power monitor                     | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ina260**               | Driver for INA260 precision digital current and power monitor                    | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ina3221**              | Driver for INA3221 shunt and bus voltage monitor                                 | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |


### Gas sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **ccs811**               | Driver for AMS CCS811 digital gas sensor                                         | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **mhz19b**               | Driver for MH-Z19B NDIR CO₂ sensor                                               | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **scd30**                | Driver for SCD30 CO₂ sensor                                                      | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **scd4x**                | Driver for SCD40/SCD41 miniature CO₂ sensor                                      | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sfa3x**                | Driver for SFA30 formaldehyde detection module (I2C)                             | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |


### GPIO expanders

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **mcp23008**             | Driver for 8-bit I2C GPIO expander MCP23008                                      | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **mcp23x17**             | Driver for I2C/SPI 16 bit GPIO expanders MCP23017/MCP23S17                       | BSD-3-Clause | esp32, esp32s2, esp32c3 | yes           |
| **pca9557**              | Driver for PCA9537/PCA9557/TCA9534 remote 4/8-bit I/O expanders for I2C-bus      | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **pcf8574**              | Driver for PCF8574 remote 8-bit I/O expander for I2C-bus                         | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **pcf8575**              | Driver for PCF8575 remote 16-bit I/O expander for I2C-bus                        | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **tca6424a**             | Driver for TCA6424A low-voltage 24-bit I2C I/O expander                          | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **tca95x5**              | Driver for TCA9535/TCA9555 remote 16-bit I/O expanders for I2C-bus               | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |


### Humidity sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **aht**                  | Driver for AHT10/AHT15/AHT20 temperature and humidity sensor                     | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **am2320**               | Driver for AM2320 temperature and humidity sensor (I2C)                          | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **bme680**               | Driver for BME680 digital environmental sensor                                   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **dht**                  | Driver for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), Itead Si7021            | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **hdc1000**              | Driver for HDC1000 temperature and humidity sensor                               | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **hts221**               | Driver for HTS221 temperature and humidity sensor                                | ISC     | esp32, esp32s2, esp32c3 | yes           |
| **sfa3x**                | Driver for SFA30 formaldehyde detection module (I2C)                             | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sht3x**                | Driver for Sensirion SHT30/SHT31/SHT35 digital temperature and humidity sensor   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sht4x**                | Driver for Sensirion SHT40/SHT41/SHT45 digital temperature and humidity sensor   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **si7021**               | Driver for Si7013/Si7020/Si7021/HTU2xD/SHT2x and compatible temperature and humidity sensors | BSD-3-Clause | esp32, esp32c3, esp8266, esp32s2, esp32c3 | yes           |


### Inertial measurement units

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **icm42670**             | Driver for TDK ICM-42670-P 6-Axis IMU                                            | ISC     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **l3gx**                 | Driver for L3Gx(L3GD20/L3G4200D) 3-axis gyroscope sensors                        | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **lsm303**               | Driver for LSM303 3-axis accelerometer and magnetometer sensor                   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **mpu6050**              | Driver for MPU6000/MPU6050 6-axis MotionTracking device                          | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |


### Input device drivers

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **button**               | HW timer-based driver for GPIO buttons                                           | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **encoder**              | HW timer-based driver for incremental rotary encoders                            | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ls7366r**              | Driver for LS7366R Quadrature Encoder Counter                                    | MIT     | esp32, esp32s2, esp32c3 | yes           |


### LED drivers

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **ht16k33**              | HT16K33 LED controller driver                                                    | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **led_strip**            | RMT-based driver for WS2812B/SK6812/APA106/SM16703 LED strips                    | MIT     | esp32, esp32s2, esp32c3 | yes           |
| **led_strip_spi**        | SPI-based driver for SK9822/APA102 LED strips                                    | MIT     | esp32, esp32c3, esp8266, esp32s2, esp32c3 | yes           |
| **max7219**              | Driver for 8-Digit LED display drivers, MAX7219/MAX7221                          | BSD-3-Clause | esp32, esp32s2, esp32c3 | yes           |


### Light sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **bh1750**               | Driver for BH1750 light sensor                                                   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **tsl2561**              | Driver for light-to-digital converter TSL2561                                    | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **tsl2591**              | Driver for light-to-digital converter TSL2591                                    | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **tsl4531**              | Driver for digital ambient light sensor TSL4531                                  | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **veml7700**             | Driver for VEML7700 ambient light sensor                                         | ISC     | esp32, esp8266, esp32s2, esp32c3 | yes           |


### Magnetic sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **hmc5883l**             | Driver for 3-axis digital compass HMC5883L and HMC5983L                          | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **lsm303**               | Driver for LSM303 3-axis accelerometer and magnetometer sensor                   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **qmc5883l**             | Driver for QMC5883L 3-axis magnetic sensor                                       | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |


### Other misc libraries

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **ds3502**               | Driver for nonvolatile digital potentiometer DS3502                              | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **example**              | An example component                                                             | ISC     | esp32, esp8266, esp32s2, esp32c3 | n/a           |
| **hd44780**              | Driver for HD44780 compatible LCD text displays                                  | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **pca9685**              | Driver for 16-channel, 12-bit PWM PCA9685                                        | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **rda5807m**             | Driver for single-chip broadcast FM radio tuner RDA5807M                         | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **tca9548**              | Driver for TCA9548A/PCA9548A low-voltage 8-channel I2C switch                    | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **tda74xx**              | Driver for TDA7439/TDA7439DS/TDA7440D audioprocessors                            | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ultrasonic**           | Driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05                       | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **wiegand**              | Wiegand protocol receiver                                                        | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |


### Pressure sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **bme680**               | Driver for BME680 digital environmental sensor                                   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **bmp180**               | Driver for BMP180 digital pressure sensor                                        | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **bmp280**               | Driver for BMP280/BME280 digital pressure sensor                                 | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **dps310**               | Driver for DPS310 barometric pressure sensor                                     | ISC     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ms5611**               | Driver for barometic pressure sensor MS5611-01BA03                               | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |


### Real-time clocks

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **ds1302**               | Driver for DS1302 RTC module                                                     | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **ds1307**               | Driver for DS1307 RTC module                                                     | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ds3231**               | Driver for DS1337 RTC and DS3231 high precision RTC module                       | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **pcf8563**              | Driver for PCF8563 (BM8563) real-time clock/calendar                             | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |


### Temperature sensors

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
| **aht**                  | Driver for AHT10/AHT15/AHT20 temperature and humidity sensor                     | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **am2320**               | Driver for AM2320 temperature and humidity sensor (I2C)                          | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **bh1900nux**            | Driver for BH1900NUX temperature sensor                                          | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **bme680**               | Driver for BME680 digital environmental sensor                                   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **bmp180**               | Driver for BMP180 digital pressure sensor                                        | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **bmp280**               | Driver for BMP280/BME280 digital pressure sensor                                 | MIT     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **dht**                  | Driver for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), Itead Si7021            | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **dps310**               | Driver for DPS310 barometric pressure sensor                                     | ISC     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ds18x20**              | Driver for DS18B20/DS18S20 families of 1-Wire temperature sensor ICs             | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | no            |
| **hdc1000**              | Driver for HDC1000 temperature and humidity sensor                               | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **hts221**               | Driver for HTS221 temperature and humidity sensor                                | ISC     | esp32, esp32s2, esp32c3 | yes           |
| **lm75**                 | Driver for LM75, a digital temperature sensor and thermal watchdog               | ISC     | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **max31725**             | Driver for MAX31725/MAX31726 temperature sensors                                 | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **max31855**             | Driver for MAX31855 cold-junction compensated thermocouple-to-digital converter  | BSD-3-Clause | esp32, esp32s2, esp32c3 | yes           |
| **max31865**             | Driver for MAX31865 resistance converter for platinum RTDs                       | BSD-3-Clause | esp32, esp32s2, esp32c3 | yes           |
| **mcp960x**              | Driver for MCP9600/MCP9601, thermocouple EMF to temperature converter            | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **mcp9808**              | Driver for MCP9808 digital temperature sensor                                    | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **ms5611**               | Driver for barometic pressure sensor MS5611-01BA03                               | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sfa3x**                | Driver for SFA30 formaldehyde detection module (I2C)                             | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sht3x**                | Driver for Sensirion SHT30/SHT31/SHT35 digital temperature and humidity sensor   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sht4x**                | Driver for Sensirion SHT40/SHT41/SHT45 digital temperature and humidity sensor   | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **si7021**               | Driver for Si7013/Si7020/Si7021/HTU2xD/SHT2x and compatible temperature and humidity sensors | BSD-3-Clause | esp32, esp32c3, esp8266, esp32s2, esp32c3 | yes           |
| **sts21**                | Driver for STS21 temperature sensor                                              | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **sts3x**                | Driver for Sensirion STS30/STS31/STS35 digital temperature sensor                | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |
| **tsys01**               | Driver for precision digital temperature sensor TSYS01                           | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 | yes           |

## Library maintainers

- [Ruslan V. Uss](https://github.com/UncleRus)
- [Tomoyuki Sakurai](https://github.com/trombik)

## Credits

- [Alex Stewart](https://github.com/astewart-consensus): `ds18x20` 
- [Alexander Bodenseher](https://github.com/saasaa): `hts221` 
- [Andrej Krutak](https://github.com/andree182): `bh1750` 
- Angelo Elias Dalzotto: `mpu6050` 
- [BernhardG](https://gitlab.com/mrnice): `ms5611` 
- [BhuvanchandraD](https://github.com/bhuvanchandra): `ds3231` 
- [Brian Schwind](https://github.com/bschwind): `tsl2561` `tsl4531` 
- [Christian Skjerning](https://github.com/slimcdk): `sts3x` 
- [David Douard](https://github.com/douardda): `mhz19b` 
- [Erriez](https://github.com/Erriez): `mhz19b` 
- [FastLED project](https://github.com/FastLED): `color` `lib8tion` `noise` 
- Frank Bargstedt: `bmp180` 
- Gabriel Boni Vicari: `mpu6050` 
- [Grupo de Pesquisa em Cultura Digital](http://gepid.upf.br/): `mpu6050` 
- GrzegorzH: `ds18x20` 
- [Gunar Schorcht](https://github.com/gschorcht): `bme680` `ccs811` `sht3x` `sts3x` 
- [Jakub Turek](https://github.com/QB4-dev): `l3gx` `lsm303` 
- [Jan Veeh](https://github.com/janveeh): `icm42670` 
- [Jeff Rowberg](https://www.i2cdevlib.com/): `mpu6050` 
- [Jose Manuel Perez](https://github.com/jmpmscorp): `lc709203f` `sgm58031` 
- [Joshua Butler](https://github.com/shuki25): `max1704x` 
- [Joshua Kallus](https://github.com/Jkallus): `ls7366r` 
- [jsuiker](https://github.com/jsuiker): `dht` 
- [Julian Doerner](https://github.com/juliandoerner): `tsl2591` 
- [Lucio Tarantino](https://github.com/dianlight): `ads111x` 
- [Manuel Markwort](https://github.com/mmarkwort): `mp2660` 
- [Marc Luehr](https://github.com/th3link): `veml7700` 
- [Nate Usher](https://github.com/nated0g): `scd30` 
- Pavel Merzlyakov: `ds1302` 
- [Raghav Jha](https://github.com/horsemann07): `mpu6050` 
- RichardA: `ds3231` 
- [Ruslan V. Uss](https://github.com/UncleRus): `ads111x` `aht` `am2320` `bh1750` `bh1900nux` `bme680` `bmp180` `bmp280` `button` `calibration` `ccs811` `dht` `ds1302` `ds1307` `ds18x20` `ds3231` `ds3502` `encoder` `framebuffer` `hd44780` `hdc1000` `hmc5883l` `hx711` `i2cdev` `ina219` `ina260` `ina3221` `led_strip` `led_strip_spi` `max31725` `max31855` `max31865` `max7219` `mcp23008` `mcp23x17` `mcp342x` `mcp4725` `mcp960x` `mcp9808` `mpu6050` `ms5611` `onewire` `pca9557` `pca9685` `pcf8563` `pcf8574` `pcf8575` `pcf8591` `qmc5883l` `rda5807m` `scd30` `scd4x` `sfa3x` `sgp40` `sht3x` `sht4x` `si7021` `sts21` `sts3x` `tca6424a` `tca9548` `tca95x5` `tda74xx` `tsl2561` `tsl4531` `tsys01` `ultrasonic` `wiegand` 
- [Sensirion AG](https://github.com/Sensirion): `scd30` `scd4x` `sfa3x` 
- [sheinz](https://github.com/sheinz): `bmp280` 
- [Thanh Pham](https://github.com/panoti): `pcf8591` 
- [Timofei Korostelev](https://github.com/chudsaviet): `ht16k33` 
- [Tomoyuki Sakurai](https://github.com/trombik): `dps310` `esp_idf_lib_helpers` `example` `led_strip_spi` `lm75` 
- [Weslley Duarte](https://github.com/weslleymfd): `ads130e08` 
- [Zaltora](https://github.com/Zaltora): `ina3221` 
- zeroday: `onewire` 
# ESP-IDF Components library

Components for Espressif ESP32 [ESP-IDF framework](https://github.com/espressif/esp-idf)

Most of them ported from [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos).


## Components

| Component      | Description                                                             | License | Thread safety
|----------------|-------------------------------------------------------------------------|---------|---------------
| **i2cdev**     | I2C utilites                                                            | MIT     | Yes
| **ds1307**     | Driver for DS1307 RTC module                                            | BSD     | Yes
| **ds3231**     | Driver for DS3231 high precision RTC module                             | MIT     | Yes
| **hmc5883l**   | Driver for HMC5883L 3-axis digital compass                              | BSD     | Yes
| **onewire**    | Bit-banging one wire driver                                             | MIT*    | No
| **ds18x20**    | Driver for DS18B20/DS18S20 families of one-wire temperature sensor ICs  | BSD     | No
| **dht**        | Driver for DHT11/DHT22 temperature and humidity sensors                 | BSD     | No
| **bmp180**     | Driver for BMP180 digital pressure sensor                               | MIT     | Yes
| **bmp280**     | Driver for BMP280 digital pressure sensor                               | MIT     | Yes
| **bh1750**     | Driver for BH1750 light sensor                                          | BSD     | Yes
| **ultrasonic** | Driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05              | BSD     | No
| **pcf8574**    | Driver for PCF8574 remote 8-bit I/O expander for I2C-bus                | MIT     | Yes
| **hd44780**    | Universal driver for HD44780 LCD display                                | BSD     | No


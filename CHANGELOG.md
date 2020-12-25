# Changelog

## v.0.7.1

#### Features

- (mcp960x) #141 Driver + example for MCP960X/L0X/RL0X
- (tsys01) #142 Driver + example for TSYS01

#### Bugfixes

- (qmc5883l) dd17522 Fix possible race condition- (tca95x5) #144 Copy-paste error, add example
- (esp_idf_lib_helpers) #143 Invalid error message
- (tsl4531) c2e835d Fix possible race condition
- (sht3x) 8289262 Fix possible race confition in SS measurement, refactoring
- (bh1750, bmp180) d57488b Fix possible race condition

## v.0.7

#### Features

- (ina260) #126 Driver for INA260 precision digital current and power monitor
- (rda5807m) #25 Driver for single-chip broadcast FM radio tuner RDA5807M
- (i2cdev) #138 I2C clock stertching support

## v.0.6.1

#### Bugfixes

- (ina219) #100 Potential error in ina219_get_gain
- (bme680) #121 Pressure calculation for bme680 gives wrong results

## v.0.6

#### Bugfixes

- (ds1307) #110 wrong squarewave frequency returned
- (sht3x, hmc5883l, hx711) #118 SHT3x measurements fail after 72min
- (pca9685) d4f5e35 Fix full on/off 
- (ina219) Typo fix

#### Features

- (ci) #116 Port CI process from Travis CI to GitHub Actions
- (ci) Update CI build tools
- (ads111x) #117 Support of ADS101x on top ADS111x
- (led_strip) #120 Smart LED strips support

## v.0.5-beta

#### Bugfixes

- (ds1302) #97 Fix critical section exit

#### Features

- (mcp342x) #92 Driver for ADC MCP3426/MCP3427/MCP3428

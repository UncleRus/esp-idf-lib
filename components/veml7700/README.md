# Driver for the VEML7700 ambient light sensor

This is the driver for the VEML7700 ambiernt light sensor from Vishay. It is not the
cheapest solution but very easy to integrate, with a minimum of further components.
The sensor can be SMD soldered by hand on a custom PCB, but also several breakout boards 
are available.

## About the sensor

The VEML7700 is a high-accuracy ambient light sensor. All measurements and filtering
is done on the chip which makes it easy to integrate. The sensitivity can be adjusted
by a gain setting and by changing the integration time. The output of the sensor is a
count value which must be converted using the resolution value in the driver. 

The sensor has power saving features which reduces the repetition of measurements.
It has also a power off feature to save even more power.

The ambient light sensor value is filterd very clone to the caracteristic of the
human eye. Besides, als the white channel with a wider wavelength spectrum. In most
applications, the ambient light value will do just fine.

## Power consumption

- 0.5 μA in shut-down mode (@3.3V)
- down to 2 μA in power save mode 4 (@3.3V)
- 45 μA on 100ms integration time (@3.3V)

## Communication interface

I2C is used as communication interface without any further interrupt pins. It has six
command codes for the settings and the output values. Read the datasheet or application
note for further information.

To reduce interactions with the integrated microcontroller, the interrupt feature can
be used. Therefore, one must configure the low and high threshold and enable the interrupt.

## Interrupt application examples

If values below a certain threshold is of interest, i.e. to activate lights when its 
getting dark outside, the low threshold should be adjusted and setting the high threshold
to maximum (65535). 

Another application could be an automated rollershutter, then both thresholds sould be
set to trigger the up and down movement of rollershutters.
 
## Measurement process

The measurement takes time and the sensor should not be read out faster than the
measurement time. Therefore the application should be adjusted to the sensor configuration
regarting integration time and power save modes. Alternatively, the interrupt feature
can be used by repeatetive reading of the interrupt status.

## Usage

This driver uses i2cdev which must be initialized first. Then initialize the device
decriptor, one discriptor per device. The sensor can be used without configuration,
but has a high chance of over-saturation on sunlight. Therefore, change gain and
integration time to your needs and configure the device.

Then, the veml7700_ambient_light and veml7700_white_channel functions can be used
to read out the brightness. The driver converts the counts from the device to lx using
the configuration.

### Hardware configurations

The driver supports multiple VEML7700 sensors at the same time that are
connected to I2C. Following figure show some possible hardware
configurations.

First figure shows the configuration with one sensor at I2C bus 0.

```text
 +------------------+   +----------+
 | ESP8266 / ESP32  |   | VEML7700 |
 |                  |   |          |
 |   GPIO 14 (SCL)  ----> SCL      |
 |   GPIO 13 (SDA)  <---> SDA      |
 +------------------+   +----------+
```

Next figure shows a possible configuration with two I2C buses.

```text
 +------------------+   +----------+
 | ESP8266 / ESP32  |   | VEML7700 |
 |                  |   |          |
 |   GPIO 14 (SCL)  ----> SCL      |
 |   GPIO 13 (SDA)  <---> SDA      |
 |                  |   +----------+
 |                  |   | VEML7700 |
 |                  |   |          |
 |   GPIO 5  (SCL)  ----> SCL      |
 |   GPIO 4  (SDA)  <---> SDA      |
 +------------------+   +----------+
```

Only one sensor per I2C bus is possible, since it uses a unconfigurable I2C slave address. 
However, one could also use GPIO controller bus buffer to connect the bus to different 
devices.

## References
[Datasheet](https://www.vishay.com/docs/84286/veml7700.pdf)

[Application Note](https://www.vishay.com/docs/84323/designingveml7700.pdf)

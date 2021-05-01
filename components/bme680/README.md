# Driver for **BME680** digital **environmental sensor**

Forked from [original driver by Gunar Schorcht](https://github.com/gschorcht/bme680-esp-idf).

The driver supports multiple BME680 sensors which are connected to the same or
different I2C interfaces with different addresses.  *SPI interface is not
supported yet*.

It is for the usage with the ESP8266 and [ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK)
or ESP32 and [ESP-IDF](https://github.com/espressif/esp-idf.git).

## About the sensor

BME680 is an ultra-low-power environmental sensor that integrates temperature,
pressure, humidity and gas sensors in only one unit.

## Communication interface

The BME680 sensor can be connected using I2C.

The I2C interface supports data rates up to 1 Mbps (ESP-IDF limitation). It is
possible to connect multiple BME680 sensors with different I2C slave addresses
to the same I2C bus or to different I2C buses. Possible I2C slave addresses
are 0x76 and 0x77.

## Measurement process

Once the BME680 has been initialized, it can be used for measurements. The
BME680 operates in two different modes, the **sleep mode** and the
**forced mode**.

The sensor starts after power-up automatically in the *sleep mode* where it
does not perform any measurement and consumes only 0.15 μA. Measurements are
only done in *forced mode*.

**Please note:** There are two further undocumented modes, the *parallel* and
the *sequential* mode. They can't be supported by the driver, since it is
not clear what they do and how to use them.

### Measurement cylce

To perform measurements, the BME680 sensor has to be triggered to switch to
the **forced mode**. In this mode, it performs exactly one measurement of
temperature, pressure, humidity, and gas in that order, the so-called
**TPHG measurement cycle**. After the execution of this TPHG measurement cycle,
**raw sensor data** become available and the sensor returns automatically back
to sleep mode.

Each of the individual measurements can be configured or skipped separately
via the sensor settings, see section **Measurement settings**. Dependent on
the configuration, the **duration of a TPHG measurement cycle** can vary from
some milliseconds up to about 4.5 seconds, especially if gas measurement is
enabled.

To avoid the blocking of the user task during measurements, the measurement
process is therefore separated into the following steps:

1. Trigger the sensor with function `bme680_force_measurement()` to switch
   to *forced mode* in which it performs exactly one THPG measurement cycle.
2. Wait the measurement duration using function `vTaskDelay()` and the value
   returned from function `bme680_get_measurement_duration()` or wait as long
   as function `bme680_is_measuring` returns true.
3. Fetch the results as fixed point values with function
   `bme680_get_results_fixed()` or as floating point values with function
   `bme680_get_results_float()`.

```C
...
// as long as sensor configuration isn't changed, the duration can be considered as constant
uint32_t duration
bme680_get_measurement_duration(sensor, &duration);
...
if (bme680_force_measurement(sensor) == ESP_OK) // STEP 1
{
    // STEP 2: passive waiting until measurement results are available
    vTaskDelay(duration);

    // STEP 3: get the results and do something with them
    if (bme680_get_results_float(sensor, &values) == ESP_OK)
        ...
}
...
```

Alternatively, busy waiting can be realized using function `bme680_is_measuring()`.

```C
...
if (bme680_force_measurement(sensor) == ESP_OK) // STEP 1
{
    // STEP 2: busy waiting until measurement results are available
    bool busy;
    do
    {
        ESP_ERROR_CHECK(bme680_is_measuring(sensor, &busy));
    }
    while(busy);

    // STEP 3: get the results and do something with them
    if (bme680_get_results_float(sensor, &values) == ESP_OK)
        ...
}
...
```

For convenience, it is also possible to use the high-level functions
`bme680_measure_float()` or `bme680_measure_fixed()`. These functions combine
all 3 steps above within a single function and are therefore very easy to use.
**Please note** that these functions must not be used when they are called
from a software timer callback function since the calling task is delayed
using function *vTaskDelay*.

```C
...
// ONE STEP: measure, wait, get the results and do something with them
if (bme680_measure_float(sensor, &values) == ESP_OK)
    ...
...
```

#### Measurement results

Once the sensor has finished the measurement raw data are available at the sensor.
Either function `bme680_get_results_fixed()` or function
`bme680_get_results_float()` can be used to fetch the results. Both functions
read raw data from the sensor and converts them into utilizable fixed point or
floating point sensor values.

**Please note:** Conversion of raw sensor data into the final sensor values is
based on very complex calculations that use a large number of calibration
parameters. Therefore, the driver does not provide functions that only return
the raw sensor data.

Dependent on sensor value representation, measurement results contain different
dimensions:

| Value          | Fixed Point   | Floating Point | Conversion
| -------------- | ------------- | -------------- |---------------------
| temperature    | 1/100 °C      | °C             | float = fixed / 100
| pressure       | Pascal        | hPascal        | float = fixed / 100
| humidity       | 1/1000 %      | %              | float = fixed / 1000
| gas_resistance | Ohm           | Ohm            | float = fixed

The gas resistance value in Ohm represents the resistance of sensor's gas sensitive
layer.

If the TPHG measurement cycle or fetching the results fails, invalid sensor values
are returned:

| Invalid Value  | Fixed Point | Floating Point
| -------------- | ----------- | ---------------
| temperature    | INT16_MIN   | -327.68
| pressure       | 0           | 0.0
| humidity       | 0           | 0.0
| gas_resistance | 0           | 0.0

## Measurement settings

The sensor allows to change a lot of measurement parameters.

### Oversampling rates

To increase the resolution of raw sensor data, the sensor supports
oversampling for temperature,  pressure, and humidity measurements. Using
function `bme680_set_oversampling_rates()`, individual **oversampling rates**
can be defined for these measurements. With an oversampling rate *osr*, the
resolution of the according raw sensor data can be increased from 16 bit to
16+ld(*osr*) bit.

Possible oversampling rates are 1x (default by the driver) 2x, 4x, 8x and 16x.
It is also possible to define an oversampling rate of 0. This **deactivates**
the corresponding measurement and the output values become invalid.

```C
...
// Changes the oversampling rate for temperature to 4x and for pressure to 2x. Humidity measurement is skipped.
ESP_ERROR_CHECK(bme680_set_oversampling_rates(sensor, osr_4x, osr_2x, osr_none));
...
```

### IIR Filter

The sensor also integrates an internal IIR filter (low pass filter) to reduce
short-term changes in sensor output values caused by external disturbances.
It effectively reduces the bandwidth of the sensor output values.

The filter can optionally be used for pressure and temperature data that are
subject to many short-term changes. With the IIR filter the resolution of
pressure and temperature data increases to 20 bit. Humidity and gas inside
the sensor does not fluctuate rapidly and does not require such a low pass
filtering.

Using function `bme680_set_filter_size()`, the user task can change the
**size of the filter**. The default size is 3. If the size of the filter
becomes 0, the filter is **not used**.

```C
...
// Change the IIR filter size for temperature and pressure to 7.
bme680_set_filter_size(sensor, iir_size_7);
...
// Don't use IIR filter
bme680_set_filter_size(sensor, iir_size_0);
...
```

### Heater profile

For the gas measurement, the sensor integrates a heater. Parameters for this
heater are defined by **heater profiles**. The sensor supports up to 10 such
heater profiles, which are numbered from 0 to 9. Each profile consists of a
temperature set-point (the target temperature) and a heating duration. By
default, only the heater profile 0 with 320 degree Celsius as target temperature
and 150 ms heating duration is defined.

**Please note:** According to the data sheet, target temperatures between 200
and 400 degrees Celsius are typical and about 20 to 30 ms are necessary for
the heater to reach the desired target temperature.

Function `bme680_set_heater_profile()` can be used to set the parameters
for one of the heater profiles 0 ... 9. Once the parameters of a heater profile
are defined, the gas measurement can be activated with that heater profile
using function `bme680_use_heater_profile()`. If -1 or `BME680_HEATER_NOT_USED`
is used as heater profile, gas measurement is deactivated completely.

```C
...
// Change the heater profile 1 to 300 degree Celsius for 100 ms and activate it
bme680_set_heater_profile (sensor, 1, 300, 100);
bme680_use_heater_profile (sensor, 1);
...
// Deactivate gas measurement completely
bme680_use_heater_profile (sensor, BME680_HEATER_NOT_USED);
...

```

If several heater profiles have been defined with function
`bme680_set_heater_profile()`, a sequence of gas measurements with different
heater parameters can be realized by a sequence of activations of different
heater profiles for successive TPHG measurements using function
`bme680_use_heater_profile()`.

For example, if there were 5 heater profiles defined with following code
during the setup

```C
bme680_set_heater_profile(sensor, 0, 200, 100);
bme680_set_heater_profile(sensor, 1, 250, 120);
bme680_set_heater_profile(sensor, 2, 300, 140);
bme680_set_heater_profile(sensor, 3, 350, 160);
bme680_set_heater_profile(sensor, 4, 400, 180);
```

the user task could use them as a sequence like following:

```C
...
while (1)
{
    switch (count++ % 5)
    {
        case 0: bme680_use_heater_profile(sensor, 0); break;
        case 1: bme680_use_heater_profile(sensor, 1); break;
        case 2: bme680_use_heater_profile(sensor, 2); break;
        case 3: bme680_use_heater_profile(sensor, 3); break;
        case 4: bme680_use_heater_profile(sensor, 4); break;
    }

    // measurement duration changes in each cycle
    uint32_t duration;
    bme680_get_measurement_duration(sensor, &duration);

    // trigger the sensor to start one TPHG measurement cycle
    if (bme680_force_measurement(sensor) == ESP_OK)
    {
        vTaskDelay(duration);

        // get the results and do something with them
        if (bme680_get_results_float(sensor, &values) == ESP_OK)
            ...
    }
    ...
}
...
```

### Ambient temperature

The heater resistance calculation algorithm takes into account the ambient
temperature of the sensor. Using function `bme680_set_ambient_temperature()`,
the ambient temperature either determined from the sensor itself or from
another temperature sensor can be set.

```C
...
bme680_set_ambient_temperature(sensor, ambient);
...
```

## Usage

First, the hardware configuration has to be established. This can differ
dependent on the communication interface and the number of sensors used.

### Hardware configurations

The driver supports multiple BME680 sensors at the same time that are
connected to I2C. Following figure show some possible hardware
configurations.

First figure shows the configuration with only one sensor at I2C bus 0.

```text
 +------------------+   +----------+
 | ESP8266 / ESP32  |   | BME680   |
 |                  |   |          |
 |   GPIO 14 (SCL)  ----> SCL      |
 |   GPIO 13 (SDA)  <---> SDA      |
 +------------------+   +----------+
```

Next figure shows a possible configuration with two I2C buses. In that case,
the sensors can have same or different I2C slave addresses.

```text
 +------------------+   +----------+
 | ESP8266 / ESP32  |   | BME680_1 |
 |                  |   |          |
 |   GPIO 14 (SCL)  ----> SCL      |
 |   GPIO 13 (SDA)  <---> SDA      |
 |                  |   +----------+
 |                  |   | BME680_2 |
 |                  |   |          |
 |   GPIO 5  (SCL)  ----> SCL      |
 |   GPIO 4  (SDA)  <---> SDA      |
 +------------------+   +----------+
```

Further configurations are possible, e.g., two sensors that are connected
at the same I2C bus with different slave addresses.


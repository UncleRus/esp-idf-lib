# FAQ

<!-- vim-markdown-toc GFM -->

* [How to debug I2C-based drivers?](#how-to-debug-i2c-based-drivers)
* [Why are semaphores (mutexes) used in i2cdev routines?](#why-are-semaphores-mutexes-used-in-i2cdev-routines)
* [How can I connect multiple I2C devices?](#how-can-i-connect-multiple-i2c-devices)
* [How can I change frequency of I2C clock? At default frequency my device is unstable or not working at all.](#how-can-i-change-frequency-of-i2c-clock-at-default-frequency-my-device-is-unstable-or-not-working-at-all)
* [How to use internal pull-up resistors](#how-to-use-internal-pull-up-resistors)
* [Can I use I2C device drivers from interrupts?](#can-i-use-i2c-device-drivers-from-interrupts)
* [Porting I2C libs to I2Cdev](#porting-i2c-libs-to-i2cdev)
* [My DHT sensor doesn't work well/doesn't work at all.](#my-dht-sensor-doesnt-work-welldoesnt-work-at-all)
* [How can I include ets_sys.h in my code without complex macros and remain compatible with different targets?](#how-can-i-include-ets_sysh-in-my-code-without-complex-macros-and-remain-compatible-with-different-targets)

<!-- vim-markdown-toc -->

## How to debug I2C-based drivers?

Common causes of I2C issues are:

* wrong wiring
* wrong pull-up resistors
* wrong I2C address
* broken I2C module
* the driver has a bug

When any of I2C-based drivers does not work, follow the steps below.

Build an [_I2C scanner_ device](examples/i2cdev/default). The device is not
necessarily an ESP device. There are many examples for various platforms.
Search by keyword `i2c scanner`.

Connect the I2C module to the I2C scanner device. Make sure appropriate
pull-up resistors are connected to `SCL` and `SDA` lines.

Scan devices on the I2C bus. If the scanner does not find the I2C device, then
your wiring might have issues. If the scanner finds the I2C device, make sure
the address found is what the driver expects. If you have more than one same
I2C modules, try them all.

If the scanner finds the I2C device and you are sure that the wiring is
correct, see the signals on the wire using an oscilloscope. Most oscilloscopes
can decode I2C signals and display I2C transactions in human-readable way.

If the driver does not work after these steps, please [let us
know](https://github.com/UncleRus/esp-idf-lib/issues).

## Why are semaphores (mutexes) used in i2cdev routines?

i2cdev uses two types of mutexes: port and transactional.

Port mutexes (there are only two of them, by the number of I2C ports) are
necessary to avoid problems in the situation mentioned in the documentation:

> The I2C APIs are not thread-safe, if you want to use one I2C port in
different tasks, you need to take care of the multi-thread issue.

They are taken before executing single I2C transactions and are released
immediately after them.

Mutexes of the second type are created one per device and are necessary if
the same device is used in several tasks:

```C
static i2c_dev_t some_dev;

void task1(void *arg)
{
     // some_dev used here
}

void task2(void *arg)
{
     // and here some_dev used too
}

...
xTaskCreate(task1, "task1", ....);
xTaskCreate(task2, "task2", ....);

```

These mutexes are used when single device operation requires several I2C
transactions in a row.

## How can I connect multiple I2C devices?

With i2cdev, you can use almost any way to connect I2C devices.

For example, in the case of using SSD1306 and two MCP3428s, I would recommend
connecting them like this:

* 2 GPIO outputs on ESP32 for dedicated screen connection via I2C bus 0
* 2 GPIO outputs to the second I2C bus, to which 2 MCP3428 are connected with
  different addresses.

If you need to connect more than one sensor with the same addresses and there
are free GPIOs, then you can directly connect these sensors to individual GPIO
outputs instead of using the I2C multiplexer. i2cdev will take care of
reconfiguring I2C driver to the according outputs when you exchange data
with these devices.

## How can I change frequency of I2C clock? At default frequency my device is unstable or not working at all.

You can change the frequency after initializing the device handler, like this:

```C
#include <esp_idf_lib_helpers.h>

...

ESP_ERROR_CHECK(ads111x_init_desc(&dev, addr, I2C_PORT, SDA_GPIO, SCL_GPIO));

#if HELPER_TARGET_IS_ESP32
    dev.cfg.master.clk_speed = 100000; // Hz
#endif

ESP_ERROR_CHECK(ads111x_set_mode(&dev, ADS111X_MODE_CONTINUOUS));    // Continuous conversion mode
ESP_ERROR_CHECK(ads111x_set_data_rate(&dev, ADS111X_DATA_RATE_32)); // 32 samples per second
...
```

## How to use internal pull-up resistors

Just enable them in `i2c_dev_t` config. For example:

```C
...

dev.cfg.scl_pullup_en = true;
dev.cfg.sda_pullup_en = true;
ESP_ERROR_CHECK(ads111x_init_desc(&dev, addr, I2C_PORT, SDA_GPIO, SCL_GPIO));

...
```

## Can I use I2C device drivers from interrupts?

With default configuration you can't. Since the drivers use mutexes, this will
crash the system.  But you can disable use of any I2C mutexes (both port and
device) in configuration: just enable CONFIG_I2CDEV_NOLOCK. Keep in mind that
after enabling this option all i2c device drivers will become non-thread safe.


## Porting I2C libs to I2Cdev

See [Porting.md](Porting.md).


## My DHT sensor doesn't work well/doesn't work at all!

1. Check if the sensor is connected correctly.
2. Use an external 4k7 pullup resistor! With an internal pullup resistor, operation will be unstable, if at all.
3. Shorten the wires that connect the sensor.
4. Use 5V for powering the sensor, not 3.3V. ESP chips are 5V tolerant.
5. Use shielded wires where the shield is connected to the GND.

## How can I include ets_sys.h in my code without complex macros and remain compatible with different targets?

Just add the `esp_idf_lib_helpers` component to the list of required ones and after that you can simply write `#include <ets_sys.h>` in your code.
See, for example, source code of `scd30` component.

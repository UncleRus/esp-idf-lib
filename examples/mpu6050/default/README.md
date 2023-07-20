# Example for `mpu6050` driver

## What it does

This example initializes an MPU-6050 connected on the I2C bus and then displays the accelerometer and gyroscope data in gravity and degrees per second in an endless loop.

**NOTE:**
Example is tested on ESP-IDF v4.4, v5.1 and ESP32 Wrover Module

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name                                    | Description           | Defaults   |
| --------------------------------------- | --------------------- | ---------- |
| `CONFIG_EXAMPLE_MPU6050_I2C_MASTER_SCL` | GPIO number for `SCL` | `esp32` 22 |
| `CONFIG_EXAMPLE_MPU6050_I2C_MASTER_SDA` | GPIO number for `SDA` | `esp32` 21 |
| `CONFIG_EXAMPLE_MPU6050_I2C_CLOCK_HZ`   | I2C Clock Freq        | 100000     |


## Logs

```
I (316) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (0) mpu6050_test: mpu6050 config: addr 0x68, sda 21, scl 22, clk, 100000 port 1
I (4341) mpu6050_test: mpu6050 connection successfull.
I (4341) mpu6050_test: **********************************************************************
I (4341) mpu6050_test: Rotation:     x=-246   y=-356   z=-52
I (4351) mpu6050_test: Acceleration: x=-16016   y=-644   z=-784
I (4351) mpu6050_test: Temperature:    33
I (5361) mpu6050_test: **********************************************************************
I (5361) mpu6050_test: Rotation:     x=-173   y=-223   z=-41
I (5361) mpu6050_test: Acceleration: x=-15872   y=-520   z=-148
I (5371) mpu6050_test: Temperature:    33
I (6371) mpu6050_test: **********************************************************************
I (6371) mpu6050_test: Rotation:     x=-318   y=268   z=-60
I (6371) mpu6050_test: Acceleration: x=-15860   y=-672   z=-172
I (6381) mpu6050_test: Temperature:    33
I (7381) mpu6050_test: **********************************************************************
I (7381) mpu6050_test: Rotation:     x=-606   y=-209   z=183
I (7381) mpu6050_test: Acceleration: x=-15616   y=-280   z=320
I (7391) mpu6050_test: Temperature:    33
I (8391) mpu6050_test: **********************************************************************
I (8391) mpu6050_test: Rotation:     x=-714   y=1262   z=-98
I (8391) mpu6050_test: Acceleration: x=-15948   y=-260   z=-60
I (8401) mpu6050_test: Temperature:    33
I (9401) mpu6050_test: **********************************************************************
I (9401) mpu6050_test: Rotation:     x=-387   y=70   z=-370
I (9401) mpu6050_test: Acceleration: x=-15972   y=-440   z=-152
I (9411) mpu6050_test: Temperature:    33
I (10411) mpu6050_test: **********************************************************************
I (10411) mpu6050_test: Rotation:     x=120   y=-288   z=-56
I (10411) mpu6050_test: Acceleration: x=-15848   y=-112   z=72
I (10421) mpu6050_test: Temperature:    33
I (11421) mpu6050_test: **********************************************************************
I (11421) mpu6050_test: Rotation:     x=-186   y=-129   z=-42
I (11421) mpu6050_test: Acceleration: x=-15844   y=-140   z=-52
I (11431) mpu6050_test: Temperature:    33
I (12431) mpu6050_test: **********************************************************************
I (12431) mpu6050_test: Rotation:     x=-184   y=-7   z=-70
I (12431) mpu6050_test: Acceleration: x=-15780   y=-128   z=16
I (12441) mpu6050_test: Temperature:    33
I (13441) mpu6050_test: **********************************************************************
I (13441) mpu6050_test: Rotation:     x=-142   y=-142   z=-49
I (13441) mpu6050_test: Acceleration: x=-15980   y=-96   z=-112
I (13451) mpu6050_test: Temperature:    33
I (14451) mpu6050_test: **********************************************************************
I (14451) mpu6050_test: Rotation:     x=-218   y=-264   z=-45
I (14451) mpu6050_test: Acceleration: x=-15864   y=-264   z=188
I (14461) mpu6050_test: Temperature:    33
I (15461) mpu6050_test: **********************************************************************
```

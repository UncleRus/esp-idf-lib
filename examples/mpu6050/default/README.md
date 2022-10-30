# Example for `mpu6050` driver

## What it does


MPU6050 is a Micro Electro-mechanical system (MEMS), it consists of three-axis accelerometer and three-axis gyroscope. It helps us to measure velocity, orientation, acceleration, displacement and other motion like features.

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
I (317) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (0) mpu6050_test: mpu6050 config: addr 0x68, sda 21, scl 22, clk, 100000 port 1
I (4342) mpu6050_test: mpu6050 connection successfull.
I (4342) mpu6050_test: **********************************************************************
I (4342) mpu6050_test: Rotation:	x=-132	y=-1046	z=782
I (4352) mpu6050_test: Acceleration:	x=7624	y=-4688	z=14480
I (4352) mpu6050_test: **********************************************************************
I (4362) mpu6050_test: temp 31
I (5362) mpu6050_test: **********************************************************************
I (5362) mpu6050_test: Rotation:	x=-725	y=201	z=85
I (5362) mpu6050_test: Acceleration:	x=7652	y=-5252	z=14076
I (5372) mpu6050_test: **********************************************************************
I (5372) mpu6050_test: temp 31
I (6382) mpu6050_test: **********************************************************************
I (6382) mpu6050_test: Rotation:	x=-861	y=-868	z=392
I (6382) mpu6050_test: Acceleration:	x=6616	y=-5840	z=13808
I (6392) mpu6050_test: **********************************************************************
I (6392) mpu6050_test: temp 31
I (7402) mpu6050_test: **********************************************************************
I (7402) mpu6050_test: Rotation:	x=-1207	y=1410	z=929
I (7402) mpu6050_test: Acceleration:	x=5712	y=-6824	z=14108
I (7412) mpu6050_test: **********************************************************************
I (7422) mpu6050_test: temp 31
I (8422) mpu6050_test: **********************************************************************
I (8422) mpu6050_test: Rotation:	x=-876	y=-169	z=876
I (8422) mpu6050_test: Acceleration:	x=3496	y=-11652	z=11276
I (8432) mpu6050_test: **********************************************************************
I (8442) mpu6050_test: temp 31
I (9442) mpu6050_test: **********************************************************************
I (9442) mpu6050_test: Rotation:	x=-774	y=-338	z=104
I (9442) mpu6050_test: Acceleration:	x=3064	y=-12676	z=10192
I (9452) mpu6050_test: **********************************************************************
I (9462) mpu6050_test: temp 31
I (10462) mpu6050_test: **********************************************************************
I (10462) mpu6050_test: Rotation:	x=-319	y=-373	z=112
I (10462) mpu6050_test: Acceleration:	x=2908	y=-13160	z=9800
I (10472) mpu6050_test: **********************************************************************
I (10482) mpu6050_test: temp 31
I (11482) mpu6050_test: **********************************************************************
I (11482) mpu6050_test: Rotation:	x=-143	y=-282	z=152
I (11482) mpu6050_test: Acceleration:	x=2768	y=-13268	z=9296
I (11492) mpu6050_test: **********************************************************************

```
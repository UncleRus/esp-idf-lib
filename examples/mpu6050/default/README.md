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
I (300) spi_flash: flash io: dio
W (303) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (318) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (0) mpu6050_test: sda 21, scl 22 clk freq 100000 addr 0x68
W (10) mpu6050_test: [mpu6050_test, 41]: mpu6050 initilized.
I (10) mpu6050_test: Testing connection...
I (20) mpu6050_test: MPU6050 connection successful
I (20) mpu6050_test: Reading internal sensor offsets...
I (30) mpu6050_test: accel x offset: -1386
I (30) mpu6050_test: accel y offset: 583
I (40) mpu6050_test: accel z offset: 1280
I (40) mpu6050_test: gyro x offset: 0
I (50) mpu6050_test: gyro y offset: 0
I (50) mpu6050_test: gyro z offset: 0
I (60) mpu6050_test: enable sensor temp
I (5060) mpu6050_test: Acceleration: x: <-6520> 	 y: <-14560> 	 z: <1992>
I (5060) mpu6050_test:     Rotation: x: <-375> 	 y: <-622> 	 z: <97>
I (5060) mpu6050_test: temperature 32
I (10070) mpu6050_test: Acceleration: x: <-6004> 	 y: <-14896> 	 z: <2404>
I (10070) mpu6050_test:     Rotation: x: <-173> 	 y: <-852> 	 z: <108>
I (10070) mpu6050_test: temperature 32
I (15070) mpu6050_test: Acceleration: x: <-5324> 	 y: <-15088> 	 z: <2860>
I (15070) mpu6050_test:     Rotation: x: <-81> 	 y: <-573> 	 z: <-175>
I (15070) mpu6050_test: temperature 32
I (20070) mpu6050_test: Acceleration: x: <-5352> 	 y: <-14916> 	 z: <3540>
I (20070) mpu6050_test:     Rotation: x: <-269> 	 y: <-362> 	 z: <-152>
I (20070) mpu6050_test: temperature 32
I (25080) mpu6050_test: Acceleration: x: <-5016> 	 y: <-14936> 	 z: <3808>
I (25080) mpu6050_test:     Rotation: x: <-455> 	 y: <-1007> 	 z: <182>
I (25080) mpu6050_test: temperature 32
I (30090) mpu6050_test: Acceleration: x: <-4336> 	 y: <-14572> 	 z: <6236>
I (30090) mpu6050_test:     Rotation: x: <50> 	 y: <-274> 	 z: <128>
I (30090) mpu6050_test: temperature 32


```
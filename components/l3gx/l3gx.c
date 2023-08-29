/*
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file l3gx.c
 *
 * ESP-IDF Driver for L3Gx 3-axis gyroscope sensor
 *
 * Copyright (c) 2023 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include "l3gx.h"

#include <esp_log.h>
#include <esp_idf_lib_helpers.h>

#define I2C_FREQ_HZ 400000 // 400kHz

/* identification number */
#define L3G4200D_WHO_AM_I_ID   0xD3
#define L3GD20_WHO_AM_I_ID     0xD4

/* registers addresses */
#define L3GX_REG_WHO_AM_I      0x0F
#define L3GX_REG_CTRL_REG1     0x20
#define L3GX_REG_CTRL_REG2     0x21
#define L3GX_REG_CTRL_REG3     0x22
#define L3GX_REG_CTRL_REG4     0x23
#define L3GX_REG_CTRL_REG5     0x24
#define L3GX_REG_REFERENCE     0x25
#define L3GX_REG_OUT_TEMP      0x26
#define L3GX_REG_STATUS_REG    0x27
#define L3GX_REG_OUT_X_L       0x28
#define L3GX_REG_OUT_X_H       0x29
#define L3GX_REG_OUT_Y_L       0x2A
#define L3GX_REG_OUT_Y_H       0x2B
#define L3GX_REG_OUT_Z_L       0x2C
#define L3GX_REG_OUT_Z_H       0x2D
#define L3GX_REG_FIFO_CTRL_REG 0x2E
#define L3GX_REG_FIFO_SRC_REG  0x2F
#define L3GX_REG_INT1_CFG      0x30
#define L3GX_REG_INT1_SRC      0x31
#define L3GX_REG_INT1_THS_XH   0x32
#define L3GX_REG_INT1_THS_XL   0x33
#define L3GX_REG_INT1_THS_YH   0x34
#define L3GX_REG_INT1_THS_YL   0x35
#define L3GX_REG_INT1_THS_ZH   0x36
#define L3GX_REG_INT1_THS_ZL   0x37
#define L3GX_REG_INT1_DURATION 0x38

#define L3GX_AUTOINCREMENT     0x80

//CTRL_REG1:
#define L3GX_ENABLE_ALL_AXES			0x07
#define L3GX_ENABLE_NORMAL_POWER_MODE	0x01
#define L3GX_DTBW_MASK					0xF0

//CTRL_REG3:
#define L3GX_ENABLE_DR_INTERRUPT		0x08

//CTRL_REG4:
#define L3GX_ENABLE_BDU					0x80
#define L3GX_SCALE_MASK					0x30

//STATUS_REG:
#define L3GX_STATUS_ZYXOR				0x80
#define L3GX_STATUS_ZOR				    0x40
#define L3GX_STATUS_YOR					0x20
#define L3GX_STATUS_XOR					0x10
#define L3GX_STATUS_ZYXDA				0x08
#define L3GX_STATUS_ZDA					0x04
#define L3GX_STATUS_YDA					0x02
#define L3GX_STATUS_XDA					0x01

static const char *TAG = "l3gx";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

inline static esp_err_t write_reg_nolock(l3gx_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1);
}

inline static esp_err_t read_reg_nolock(l3gx_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(&dev->i2c_dev, reg, val, 1);
}

static esp_err_t update_reg_nolock(l3gx_t *dev, uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t v;
    CHECK(read_reg_nolock(dev, reg, &v));
    CHECK(write_reg_nolock(dev, reg, (v & ~mask) | val));
    return write_reg_nolock(dev, reg, v);
}

esp_err_t l3gx_init_desc(l3gx_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    dev->sensor_type = L3GX_TYPE_UNKNOWN;
    dev->scale = L3GX_SCALE_250;
    dev->datarate_bandwith = L3GX_DRBW_100_125;

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t l3gx_init(l3gx_t *dev)
{
	CHECK_ARG(dev);
	int type;
	uint8_t v = 0;

	const uint8_t expected_ids[] = {
		[L3GX_TYPE_L3G4200D] = L3G4200D_WHO_AM_I_ID,
		[L3GX_TYPE_L3GD20] = L3GD20_WHO_AM_I_ID
	};
	const char* sensor_type_str[] = {
		[L3GX_TYPE_L3G4200D] = "L3G4200D",
		[L3GX_TYPE_L3GD20] = "L3GD20"
	};

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, L3GX_REG_WHO_AM_I, &v, 1));

	for(type=0; type < L3GX_TYPE_UNKNOWN; type++){
		if(v == expected_ids[type]){
			dev->sensor_type = type;
			ESP_LOGI(TAG, "sensor %s connected", sensor_type_str[dev->sensor_type]);
			break;
		}
	}
	if(type == L3GX_TYPE_UNKNOWN){
		I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
		ESP_LOGE(TAG, "Unknown ID: 0x%02x", v);
		return ESP_ERR_NOT_FOUND;
	}

    /* setup CTRL_REG1
		[7:6] DR - Output data rate
		[5:4] BW - Bandwidth
		[3] PD - Power-down mode
		[2] ZEN - Z axis enable
		[1] YEN - Y axis enable
		[0] XEN - X axis enable
	*/

	v = 0x00;
	v |= (dev->datarate_bandwith << 4);
	v |= (L3GX_ENABLE_NORMAL_POWER_MODE << 3);
	v |= (L3GX_ENABLE_ALL_AXES << 0);
	I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, L3GX_REG_CTRL_REG1, &v, 1));

    /* setup CTRL_REG2
	[7:6] Reserved
		[5:4] HPM1, HPM0 - High Pass filter Mode Selection, Default value: 00:
				00 Normal mode (reset reading HP_RESET_FILTER)
				01 Reference signal for filtering
				10 Normal mode
				11 Autoreset on interrupt event
		[3:0] HPCF3, HPCF2, HPCF1, HPCF0 - High Pass filter Cut Off frequency selection:
				HPCF3	ODR= 100 Hz ODR= 200 Hz ODR= 400 Hz ODR= 800 Hz
				0000	8			15			30			56
				0001	4			8			15			30
				0010	2			4			8			15
				0011	1			2			4			8
				0100	0.5			1			2			4
				0101	0.2			0.5			1			2
				0110	0.1			0.2			0.5			1
				0111	0.05		0.1			0.2			0.5
				1000	0.02		0.05		0.1			0.2
				1001	0.01		0.02		0.05		0.1
	*/

	v = 0x00;
	I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, L3GX_REG_CTRL_REG2, &v, 1));

    /* setup CTRL_REG3
		[7] I1_Int1		Interrupt enable on INT1 pin.				Default value: 0. (0: Disable; 1: Enable)
		[6] I1_Boot		Boot status available on INT1.				Default value: 0. (0: Disable; 1: Enable)
		[5] H_Lactive	Interrupt active configuration on INT1.		Default value: 0. (0: High; 1:Low)
		[4] PP_OD		Push- Pull / Open drain.					Default value: 0. (0: Push- Pull; 1: Open drain)
		[3] I2_DRDY		Date Ready on DRDY/INT2.					Default value: 0. (0: Disable; 1: Enable)
		[2] I2_WTM		FIFO Watermark interrupt on DRDY/INT2.		Default value: 0. (0: Disable; 1: Enable)
		[1] I2_ORun		FIFO Overrun interrupt on DRDY/INT2			Default value: 0. (0: Disable; 1: Enable)
		[0] I2_Empty	FIFO Empty interrupt on DRDY/INT2.			Default value: 0. (0: Disable; 1: Enable)
	*/
	v = 0x00;
	I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, L3GX_REG_CTRL_REG3, &v, 1));


    /* setup CTRL_REG4
    	[7]		BDU			- Block Data Update.						Default value: 0	(0: continous update; 1: output registers not updated until MSB and LSB reading)
		[6]		BLE			- Big/Little Endian Data Selection.			Default value: 0.
		[5:4]	FS1 FS0		- Full Scale selection.						Default value: 00	(00: 250 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps) ST1-ST0 Self Test Enable. Default value: 00
					+/- 250DPS		-> 8.75 mDPS/LSB
					+/- 1000DPS		-> 17.5 mDPS/LSB
					+/- 2000DPS		-> 70.0 mDPS/LSB
		[3]		RESERVED	-
		[2:1]	ST1 ST0		- Self Test Enable.							Default value: 00	(00: Self Test Disabled;)
		[0]		SIM			- SIM SPI Serial Interface Mode selection.	Default value: 0	(0: 4-wire interface; 1: 3-wire interface).

    */
	v = 0x00;
	v |= L3GX_ENABLE_BDU;
    v |= (dev->scale << 4);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, L3GX_REG_CTRL_REG4, &v, 1));

    /* setup CTRL_REG5
     	[7] BOOT						Reboot memory content.			Default value: 0 (0: normal mode; 1: reboot memory content)
		[6] FIFO_EN						FIFO_EN FIFO enable.			Default value: 0 (0: FIFO disable; 1: FIFO Enable)
		[5] RESERVED					-
		[4] HPen						HPen High Pass filter Enable.	Default value: 0 (0: HPF disabled; 1: HPF enabled. See Figure 20)
		[3:2] INT1_Sel1,	INT1_Sel0	INT1 selection configuration.	Default value: 0
		[1:0] Out_Sel1, Out_Sel0		Out selection configuration.	Default value: 0
    */
	v = 0x00;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, L3GX_REG_CTRL_REG5, &v, 1));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t l3gx_free_desc(l3gx_t *dev)
{
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t l3gd20_get_chip_id(l3gx_t *dev, uint8_t *id)
{
    CHECK_ARG(dev && id);
	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, L3GX_REG_WHO_AM_I, id));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t l3gd20_set_scale(l3gx_t *dev, l3gx_scale_t scale)
{
	CHECK_ARG(dev && scale >= L3GX_SCALE_250 && scale <= L3GX_SCALE_2000);
	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	dev->scale = scale;
	I2C_DEV_CHECK(&dev->i2c_dev, update_reg_nolock(dev,L3GX_REG_CTRL_REG4,(dev->scale << 4),L3GX_SCALE_MASK));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
	return ESP_OK;
}

esp_err_t l3gd20_set_datarate_and_bandwith(l3gx_t *dev, l3gx_drbw_t drbw)
{
	CHECK_ARG(dev && drbw >= L3GX_DRBW_100_125 && drbw <= L3GX_DRBW_800_110);
	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	dev->datarate_bandwith = drbw;
	I2C_DEV_CHECK(&dev->i2c_dev, update_reg_nolock(dev,L3GX_REG_CTRL_REG1,(dev->datarate_bandwith << 4),L3GX_DTBW_MASK));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
	return ESP_OK;
}

esp_err_t l3gx_data_ready(l3gx_t *dev, bool *ready)
{
	CHECK_ARG(dev && ready);

	uint8_t v;
	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, L3GX_REG_STATUS_REG, &v));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
	*ready = v & L3GX_STATUS_ZYXDA;
	return ESP_OK;
}

esp_err_t l3gx_get_raw_data(l3gx_t *dev, l3gx_raw_data_t *raw)
{
	CHECK_ARG(dev && raw);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev, L3GX_REG_OUT_X_L|L3GX_AUTOINCREMENT, raw, 6);
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "Could not read data register, err = %d", ret);
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
	return ret;
}

esp_err_t l3gd20_raw_to_dps(l3gx_t *dev, l3gx_raw_data_t *raw, l3gx_data_t *data)
{
	/* sensitivity factors, datasheet pg. 9 */
	const float sensivity_factors[] = {
		[L3GX_SCALE_250] =  0.00875, // 8.75 mdps/digit
		[L3GX_SCALE_500] =  0.0175,  // 17.5 mdps/digit
		[L3GX_SCALE_2000] = 0.0700   // 70.0 mdps/digit
	};
	data->x = raw->x * sensivity_factors[dev->scale];
	data->y = raw->y * sensivity_factors[dev->scale];
	data->z = raw->z * sensivity_factors[dev->scale];
	return ESP_OK;
}

esp_err_t l3gx_get_data(l3gx_t *dev, l3gx_data_t *data)
{
	CHECK_ARG(dev && data);

	l3gx_raw_data_t raw;
	CHECK(l3gx_get_raw_data(dev, &raw));
	return l3gd20_raw_to_dps(dev, &raw, data);
}

esp_err_t l3gx_get_raw_temp(l3gx_t *dev, int16_t *temp)
{
	CHECK_ARG(dev && temp);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev, L3GX_REG_OUT_TEMP|L3GX_AUTOINCREMENT, temp, 2);
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "Could not read OUT_TEMP register, err = %d", ret);
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
	return ret;
}

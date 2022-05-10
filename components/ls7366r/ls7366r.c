/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Joshua Kallus <joshk.kallus3@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ls7366r.c
 *
 * ESP-IDF driver for LS7366R Quadrature Encoder Counter
 *
 * Datasheet: https://lsicsi.com/datasheets/LS7366R.pdf
 *
 * Copyright (c) 2021 Joshua Kallus <joshk.kallus3@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "ls7366r.h"
#include <string.h>

// Registers
#define REG_MDR0 0x08
#define REG_MDR1 0x10
#define REG_DTR 0x18
#define REG_CNTR 0x20
#define REG_OTR 0x28
#define REG_STR 0x30

// Commands
#define CMD_CLR 0x00
#define CMD_WR 0x80
#define CMD_LOAD 0xC0
#define CMD_RD 0x40

// Mode Bitmasks

#define COUNTER_NON_QUAD              0x00    
#define COUNTER_1X_QUAD               0x01    
#define COUNTER_2X_QUAD               0x02    
#define COUNTER_4X_QUAD               0x03    

#define COUNTER_FREE_RUN              0x00    
#define COUNTER_SINGLE_COUNT          0x04    
#define COUNTER_RANGE_LIMIT           0x08    
#define COUNTER_N_MODULE              0x0C   

#define COUNTER_INDEX_DISABLED        0x00    
#define COUNTER_INDEX_LOAD_CNTR       0x10     
#define COUNTER_INDEX_RESET_CNTR      0x20     
#define COUNTER_INDEX_LOAD_OTR        0x30     

#define COUNTER_INDEX_ASYNC           0x00    
#define COUNTER_INDEX_SYNC            0x40    

#define COUNTER_FILTER_CLOCK_DIV1     0x00    
#define COUNTER_FILTER_CLOCK_DIV2     0x80    

#define COUNTER_MODE_32               0x00    
#define COUNTER_MODE_24               0x01    
#define COUNTER_MODE_16               0x02    
#define COUNTER_MODE_8                0x03    

#define COUNTER_ENABLE                0x00    
#define COUNTER_DISABLE               0x04    

#define COUNTER_FLAG_IDX              0x10    
#define COUNTER_FLAG_CMP              0x20    
#define COUNTER_FLAG_BW               0x40    
#define COUNTER_FLAG_CY               0x80    

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


// Interface functions //////////////////////////////////////////////////////////////
static esp_err_t write_command(spi_device_handle_t spi, uint8_t command)
{
	esp_err_t ret;
	spi_transaction_t trans = {
		.cmd = command,
		.length = 0,
		.tx_buffer = NULL,
		.rx_buffer = NULL
	};
	
	spi_device_acquire_bus(spi, portMAX_DELAY);
	ret = spi_device_transmit(spi, &trans);
	spi_device_release_bus(spi);
	return ret;
}

static esp_err_t write_data(spi_device_handle_t spi, uint8_t command, uint8_t* data, uint8_t count)
{
	esp_err_t ret;
	spi_transaction_t trans = { 
		.cmd = command,
		.length = count * 8,
		.flags = SPI_TRANS_USE_TXDATA,
		.rx_buffer = NULL,
	};
	
	memcpy(trans.tx_data, data, count);
	
	spi_device_acquire_bus(spi, portMAX_DELAY);
	ret = spi_device_transmit(spi, &trans);
	spi_device_release_bus(spi);
	return ret;
}

static esp_err_t read_register(spi_device_handle_t spi, uint8_t command, uint8_t *data)
{
	esp_err_t ret;
	spi_transaction_t trans = {
		.cmd = command,
		.tx_buffer = NULL,
		.rxlength = 1 * 8,
		.flags = SPI_TRANS_USE_RXDATA,
		.length = 8
	};
	
	spi_device_acquire_bus(spi, portMAX_DELAY);
	ret = spi_device_transmit(spi, &trans);
	spi_device_release_bus(spi);
	
	*data = trans.rx_data[0];
	return ret;
}

static esp_err_t read_data(spi_device_handle_t spi, uint8_t command, uint8_t* data, uint8_t count)
{
	esp_err_t ret;
	spi_transaction_t trans = {
		.cmd = command,
		.flags = SPI_TRANS_USE_RXDATA,
		.rxlength = count * 8,
		.tx_buffer = NULL,
		.length = 8 * count,
	};
	
	spi_device_acquire_bus(spi, portMAX_DELAY);
	ret = spi_device_transmit(spi, &trans);
	spi_device_release_bus(spi);
	
	memcpy(data, trans.rx_data, count);
	return ret;
}
//////////////////////////////////////////////////////////////////////////////////////

///////////////// Control Functions ///////////////////////////////////////////////////
static esp_err_t write_mdr0(spi_device_handle_t spi, uint8_t settings)
{
	return write_data(spi, CMD_WR | REG_MDR0, &settings, 1);
}

static esp_err_t write_mdr1(spi_device_handle_t spi, uint8_t settings)
{
	return write_data(spi, CMD_WR | REG_MDR1, &settings, 1);
}

static esp_err_t write_dtr(spi_device_handle_t spi, uint8_t* data, uint8_t length)
{
	return write_data(spi, CMD_WR | REG_DTR, data, length);
}

static esp_err_t read_mdr1(spi_device_handle_t spi, uint8_t *data)
{
	return read_register(spi, CMD_RD | REG_MDR1, data);
}

static esp_err_t read_cntr(spi_device_handle_t spi, int32_t* data)
{
	esp_err_t ret;
	uint8_t data_buf[4];
	uint32_t result;
	ret = read_data(spi, CMD_RD | REG_CNTR, data_buf, 4);
	result = data_buf[0];
	
	for (uint8_t cnt = 1; cnt < 4; cnt++)
	{
		result <<= 8;
		result |= data_buf[cnt];
	}
	
	*data = result;
	return ret;
}

static esp_err_t clear_cntr(spi_device_handle_t spi)
{
	return write_command(spi, CMD_CLR | REG_CNTR);
}

static esp_err_t clear_str(spi_device_handle_t spi)
{
	return write_command(spi, CMD_CLR | REG_STR);
}

static esp_err_t counter_enable(spi_device_handle_t spi)
{
	esp_err_t ret;
	uint8_t mdr1_current;
	ret = read_mdr1(spi, &mdr1_current);
	if (ret != ESP_OK)
		return ret;
	ret = write_mdr1(spi, mdr1_current & 0xFB); // bit 3 of MDR1 is 0 enable, 1 disable 0xFB = 0b11111011 masks all but the bit we need to set to 0					
	return ret;	
}

static esp_err_t counter_disable(spi_device_handle_t spi)
{
	esp_err_t ret;
	uint8_t mdr1_current;
	ret = read_mdr1(spi, &mdr1_current);
	if (ret != ESP_OK)
		return ret;
	ret = write_mdr1(spi, mdr1_current | 0x04); // bit 3 of MDR1 is 0 enable, 1 disable 0x04 = 0b00000100 masks all but the bit we need to set to 1	
	return ret;
	
}
///////////////////////////////////////////////////////////////////////////////////////////


esp_err_t ls7366r_init_desc(ls7366r_t *dev, spi_host_device_t host, uint32_t clock_speed_hz, gpio_num_t cs_pin)
{
	CHECK_ARG(dev);
	memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
	dev->spi_cfg.spics_io_num = cs_pin;
	dev->spi_cfg.clock_speed_hz = clock_speed_hz;
	dev->spi_cfg.mode = 0;
	dev->spi_cfg.queue_size = 1;
	dev->spi_cfg.command_bits = 8;
	return spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
}

esp_err_t ls7366r_free_desc(ls7366r_t *dev)
{
	CHECK_ARG(dev);
	return spi_bus_remove_device(dev->spi_dev);
}

esp_err_t ls7366r_set_config(ls7366r_t *dev, const ls7366r_config_t *config)
{
	esp_err_t ret;
	
	// Set MDR0
	uint8_t mdr0_set = 0x00;
	switch (config->filter_clock_divider)
	{
		case LS7366R_FILTER_CLK_1:
			mdr0_set |= COUNTER_FILTER_CLOCK_DIV1;
			break;
		case LS7366R_FILTER_CLK_2:
			mdr0_set |= COUNTER_FILTER_CLOCK_DIV2;
			break;
	}
	
	switch (config->index_sync)
	{	
		case LS7366R_INDEX_SYNCHRONOUS:
			mdr0_set |= COUNTER_INDEX_SYNC;
			break;
		case LS7366R_INDEX_ASYNCHRONOUS:
			mdr0_set |= COUNTER_INDEX_ASYNC;
			break;
	}
	
	switch (config->index_mode)
	{
		case LS7366R_INDEX_DISABLED:
			mdr0_set |= COUNTER_INDEX_DISABLED;
			break;
		case LS7366R_INDEX_LOAD_CNTR:
			mdr0_set |= COUNTER_INDEX_LOAD_CNTR;
			break;
		case LS7366R_INDEX_RESET_CNTR:
			mdr0_set |= COUNTER_INDEX_RESET_CNTR;
			break;
		case LS7366R_INDEX_LOAD_OTR:
			mdr0_set |= COUNTER_INDEX_LOAD_OTR;
			break;
	}
	
	switch (config->count_type)
	{
		case LS7366R_NON_QUAD:
			mdr0_set |= COUNTER_NON_QUAD;
			break;
		case LS7366R_1x_QUAD:
			mdr0_set |= COUNTER_1X_QUAD;
			break;
		case LS7366R_2x_QUAD:
			mdr0_set |= COUNTER_2X_QUAD;
			break;
		case LS7366R_4X_QUAD:
			mdr0_set |= COUNTER_4X_QUAD;
			break;
	}
	
	uint8_t mdr1_set = 0x00;
	
	uint8_t flag_borrow = config->flag_mode.borrow == LS7366R_FLAG_BORROW_ENABLE ? COUNTER_FLAG_BW : 0x00;
	uint8_t flag_index = config->flag_mode.index == LS7366R_FLAG_INDEX_ENABLE ? COUNTER_FLAG_IDX : 0x00;
	uint8_t flag_compare = config->flag_mode.compare == LS7366R_FLAG_COMPARE_ENABLE ? COUNTER_FLAG_CMP : 0x00;
	uint8_t flag_carry = config->flag_mode.carry == LS7366R_FLAG_CARRY_ENABLE ? COUNTER_FLAG_CY : 0x00;
	
	mdr1_set = mdr1_set | flag_borrow | flag_carry | flag_compare | flag_index;
	
	switch (config->counter_enable)
	{
		case LS7366R_COUNTER_ENABLE:
			mdr1_set |= COUNTER_ENABLE;
			break;
		case LS7366R_COUNTER_DISABLE:
			mdr1_set |= COUNTER_DISABLE;
			break;
	}
	
	switch (config->counter_bits)
	{
		case LS7366R_8_BIT:
			mdr1_set |= COUNTER_MODE_8;
			break;
		case LS7366R_16_BIT:
			mdr1_set |= COUNTER_MODE_16;
			break;
		case LS7366R_24_BIT:
			mdr1_set |= COUNTER_MODE_24;
			break;
		case LS7366R_32_BIT:
			mdr1_set |= COUNTER_MODE_32;
			break;
	}
	
	ret = clear_cntr(dev->spi_dev);
	if (ret != ESP_OK)
		return ret;
	ret = clear_str(dev->spi_dev);
	if (ret != ESP_OK)
		return ret;
	ret = write_mdr0(dev->spi_dev, mdr0_set);
	if (ret != ESP_OK)
		return ret;
	ret = write_mdr1(dev->spi_dev, mdr1_set);
	return ret;
}

esp_err_t ls7366r_get_count(ls7366r_t *dev, int32_t *count)
{
	return read_cntr(dev->spi_dev, count);	
}

esp_err_t ls7366r_set_compare_val(ls7366r_t *dev, int32_t cmp)
{
	uint8_t cmp_data[4];
	uint8_t* ptr = (uint8_t*)(&cmp);
	cmp_data[3] = ptr[0];
	cmp_data[2] = ptr[1];
	cmp_data[1] = ptr[2];
	cmp_data[0] = ptr[3];
	
	return write_dtr(dev->spi_dev, (uint8_t*)&cmp_data, 4);
}

esp_err_t ls7366r_clear_counter(ls7366r_t *dev)
{
	return clear_cntr(dev->spi_dev);
}	

esp_err_t ls7366r_counter_enable(ls7366r_t *dev, bool enable)
{
	return enable ? counter_enable(dev->spi_dev) : counter_disable(dev->spi_dev);
}

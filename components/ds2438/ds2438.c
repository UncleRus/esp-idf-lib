
/*
 * The MIT License (MIT)
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
 * ----------------------------------------------------------------------------
 * Portions copyright (C) 2000 Dallas Semiconductor Corporation, under the
 * following additional terms:
 *
 * Except as contained in this notice, the name of Dallas Semiconductor
 * shall not be used except as stated in the Dallas Semiconductor
 * Branding Policy.
 */


/* Kernal headers */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* esp_idf includes */
#include <esp_log.h>
#include <esp_err.h>
#include <esp_utils.h>
#include <ets_sys.h>
#include "ds2438.h"

#define TAG "ds2438"

// Dallas family 
#define FAMILY_DS2438               0x26

// Memory commands 
#define DS2438_READ_SCRATCH		    0xBE
#define DS2438_WRITE_SCRATCH		0x4E
#define DS2438_COPY_SCRATCH		    0x48
#define DS2438_RECALL_MEMORY		0xB8

// Register commands
#define DS2438_CONVERT_TEMP		    0x44
#define DS2438_CONVERT_VOLTAGE	    0xB4

#define DS2438_ADC_INPUT_VAD        0
#define DS2438_ADC_INPUT_VDD        1

#define DS2438_CHANNEL_A            DS2438_ADC_INPUT_VAD
#define DS2438_CHANNEL_B            DS2438_ADC_INPUT_VDD

#define DS2438_PAGE_SIZE		    8
#define DS2438_MAX_CONVERSION_TIME	10		/* ms */

// Page #0 definitions
#define DS2438_STATUS_REG		0x00		/* Status/Configuration Register */
#define DS2438_STATUS_IAD		(1 << 0)	/* Current A/D Control Bit */
#define DS2438_STATUS_CA		(1 << 1)	/* Current Accumulator Configuration */
#define DS2438_STATUS_EE		(1 << 2)	/* Current Accumulator Shadow Selector bit */
#define DS2438_STATUS_AD		(1 << 3)	/* Voltage A/D Input Select Bit */
#define DS2438_STATUS_TB		(1 << 4)	/* Temperature Busy Flag */
#define DS2438_STATUS_NVB		(1 << 5)	/* Nonvolatile Memory Busy Flag */
#define DS2438_STATUS_ADB		(1 << 6)	/* A/D Converter Busy Flag */

#define DS2438_TEMP_LSB			0x01
#define DS2438_TEMP_MSB			0x02
#define DS2438_VOLTAGE_LSB		0x03
#define DS2438_VOLTAGE_MSB		0x04
#define DS2438_CURRENT_LSB		0x05
#define DS2438_CURRENT_MSB		0x06
#define DS2438_THRESHOLD		0x07

// Page #1 definitions
#define DS2438_ETM_0			0x00
#define DS2438_ETM_1			0x01
#define DS2438_ETM_2			0x02
#define DS2438_ETM_3			0x03
#define DS2438_ICA			    0x04
#define DS2438_OFFSET_LSB		0x05
#define DS2438_OFFSET_MSB

static uint8_t _uniqueID[8] = {0};
/* ----------------------------------------------------- */
esp_err_t ds2438_get_temperature(const ds2438_dev_t *ds2438_conf, double *temp)
{
    ESP_PARAM_CHECK(ds2438_conf);
    esp_err_t ret = ESP_OK;
    ds2438_data_t data = {0};
    *temp = 0;
    ret = ds2438_update(ds2438_conf, &(data));
    ESP_ERROR_RETURN(ret != ESP_OK, ret, "Failed to get temperature");

    *temp = data.voltage;
    return ret;
}

/* ----------------------------------------------------- */
esp_err_t ds2438_get_voltage(const ds2438_dev_t *ds2438_conf, float *voltage)
{
    ESP_PARAM_CHECK(ds2438_conf);
    esp_err_t ret = ESP_OK;
    ds2438_data_t data = {0};
    *voltage = 0;
    ret = ds2438_update(ds2438_conf, &(data));
    ESP_ERROR_RETURN(ret != ESP_OK, ret, "Failed to get voltage");

    *voltage = data.voltage;
    return ret;
}

/* ----------------------------------------------------- */
static esp_err_t ds2438_write_page_zero(const ds2438_dev_t *ds2438_conf, uint8_t *data)
{
    ESP_PARAM_CHECK(data);
    ESP_PARAM_CHECK(ds2438_conf);

    onewire_reset(ds2438_conf->onewire_pin);
    onewire_select(ds2438_conf->onewire_pin, ds2438_conf->addr);
    onewire_write(ds2438_conf->onewire_pin, DS2438_WRITE_SCRATCH);
    onewire_write(ds2438_conf->onewire_pin, DS2438_STATUS_REG);
    for (int i = 0; i < 8; i++)
    {
        onewire_write(ds2438_conf->onewire_pin, data[i]);
    }
    onewire_reset(ds2438_conf->onewire_pin);
    onewire_select(ds2438_conf->onewire_pin, ds2438_conf->addr);

    onewire_write(ds2438_conf->onewire_pin, DS2438_COPY_SCRATCH);
    onewire_write(ds2438_conf->onewire_pin, DS2438_STATUS_REG);
    return ESP_OK;
}

/* ----------------------------------------------------- */
static esp_err_t ds2438_read_page_zero(const ds2438_dev_t *ds2438_conf, uint8_t *data)
{
    ESP_PARAM_CHECK(data);
    ESP_PARAM_CHECK(ds2438_conf);

    onewire_reset(ds2438_conf->onewire_pin);
    onewire_select(ds2438_conf->onewire_pin, ds2438_conf->addr);
    onewire_write(ds2438_conf->onewire_pin, DS2438_RECALL_MEMORY);
    onewire_write(ds2438_conf->onewire_pin, DS2438_STATUS_REG);
    onewire_reset(ds2438_conf->onewire_pin);
    onewire_select(ds2438_conf->onewire_pin, ds2438_conf->addr);
    onewire_write(ds2438_conf->onewire_pin, DS2438_READ_SCRATCH);
    onewire_write(ds2438_conf->onewire_pin, DS2438_STATUS_REG);
    if (!onewire_read_bytes(ds2438_conf->onewire_pin, data, 9))
    {
        return ESP_FAIL;
    }
    uint8_t crc = onewire_crc8(data, 8);
    if (crc != data[8])
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ----------------------------------------------------- */
static bool ds2438_select_channel(const ds2438_dev_t *ds2438_conf, int channel)
{
    uint8_t data[9];
    if (ds2438_read_page_zero(ds2438_conf, (uint8_t *)data) != ESP_OK)
    {
        return false;
    }

    if ((ds2438_conf->mode == DS2438_MODE_CHA) || 
        (ds2438_conf->mode == DS2438_MODE_TEMPERATURE))
    {
        data[0] = data[0] | 0x08;
    }
    else
    {
        data[0] = data[0] & 0xf7;
    }

    ds2438_write_page_zero(ds2438_conf, (uint8_t *)data);
    return true;
}

/* ----------------------------------------------------- */
static bool ds2438_start_conversion(const ds2438_dev_t *ds2438_conf, int channel, bool doTemperature)
{
    if (!ds2438_select_channel(ds2438_conf, channel))
    {
        return false;
    }

    onewire_reset(ds2438_conf->onewire_pin);
    onewire_select(ds2438_conf->onewire_pin, ds2438_conf->addr);

    if (doTemperature)
    {
        onewire_write(ds2438_conf->onewire_pin, DS2438_CONVERT_TEMP);
        vTaskDelay(DS2438_MAX_CONVERSION_TIME);
        onewire_reset(ds2438_conf->onewire_pin);
        onewire_select(ds2438_conf->onewire_pin, ds2438_conf->addr);
    }
    onewire_write(ds2438_conf->onewire_pin, DS2438_CONVERT_VOLTAGE);
    vTaskDelay(DS2438_MAX_CONVERSION_TIME);
    return true;
}

/* ----------------------------------------------------- */
onewire_addr_t ds2438_get_device_addr(const ds2438_dev_t *ds2438_conf)
{
    onewire_search_t device;
    onewire_search_start(&(device));
    onewire_search_prefix(&(device), FAMILY_DS2438);
    onewire_addr_t addr = onewire_search_next(&(device), ds2438_conf->onewire_pin);
    if (device.last_device_found)
    {
        for (int i = 7, j = 0; i >= 0 && j < 8; i--, j++)
        {
            _uniqueID[j] = device.rom_no[i];
        }
    }
    return addr;
}

/* ----------------------------------------------------- */
esp_err_t ds2438_get_uniqueID(uint8_t *id)
{
    ESP_PARAM_CHECK(id);
    for (size_t i = 0; i < 8; i++)
    {
        id[i] = _uniqueID[i];
    }
    return ESP_OK;
}
/* ----------------------------------------------------- */
esp_err_t ds2438_update(const ds2438_dev_t *ds2438_conf, ds2438_data_t *ds2438_read)
{
    uint8_t data[9];
    if ((ds2438_conf->mode & DS2438_MODE_CHA) || (ds2438_conf->mode == DS2438_MODE_TEMPERATURE))
    {
        bool doTemperature = ds2438_conf->mode & DS2438_MODE_TEMPERATURE;
        if (!ds2438_start_conversion(ds2438_conf, DS2438_CHANNEL_A, doTemperature))
        {
            ESP_ERROR_RETURN(ESP_FAIL, ESP_FAIL, "Failed to start conversion.");
        }
        if (ds2438_read_page_zero(ds2438_conf, (uint8_t *)data) != ESP_OK)
        {
            ESP_ERROR_RETURN(ESP_FAIL, ESP_FAIL, "Failed to read page zero.");
        }

        if (doTemperature)
        {
            ds2438_read->temperatue = (double)(((((int16_t)data[2]) << 8) | (data[1] & 0x0ff)) >> 3) * 0.03125;
        }
        if (ds2438_conf->mode & DS2438_CHANNEL_A)
        {
            ds2438_read->voltage = (((data[4] << 8) & 0x00300) | (data[3] & 0x0ff)) / 100.0;
        }
    }
    if (ds2438_conf->mode & DS2438_CHANNEL_B)
    {
        bool doTemperature = ds2438_conf->mode & DS2438_MODE_TEMPERATURE && !(ds2438_conf->mode & DS2438_CHANNEL_A);
        if (!ds2438_start_conversion(ds2438_conf, DS2438_CHANNEL_B, doTemperature))
        {
            ESP_ERROR_RETURN(ESP_FAIL, ESP_FAIL, "Failed to start conversion.");
        }
        if (ds2438_read_page_zero(ds2438_conf, (uint8_t *)data) != ESP_OK)
        {
            ESP_ERROR_RETURN(ESP_FAIL, ESP_FAIL, "Failed to read page zero.");
        }

        if (doTemperature)
        {
            ds2438_read->temperatue = (double)(((((int16_t)data[2]) << 8) | (data[1] & 0x0ff)) >> 3) * 0.03125;
        }
        ds2438_read->voltage = (((data[4] << 8) & 0x00300) | (data[3] & 0x0ff)) / 100.0;
    }
    else
    {
        ESP_LOGE(TAG, "Invalid ds2438_conf->mode <%d>.", ds2438_conf->mode);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ----------------------------------------------------- */
esp_err_t ds2438_init(ds2438_dev_t *ds2438_conf)
{
    ESP_PARAM_CHECK(ds2438_conf);
    onewire_reset(ds2438_conf->onewire_pin);
    ds2438_conf->addr = ds2438_get_device_addr(ds2438_conf);
    if (ds2438_conf->addr == ONEWIRE_NONE)
    {
        ESP_ERROR_RETURN(ESP_FAIL, ESP_FAIL, "Opps! device not found.");
    }
    return ESP_OK;
}

/* ----------------------------------------------------- */

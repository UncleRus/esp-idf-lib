#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include "i2cdev.h"
#include "i2cdev_lock.h"

#define PROBE_XFER_TIMEOUT_MS   (100)

static const char *TAG = "i2cdev";

static i2c_port_state_t states[I2C_NUM_MAX];

inline static bool cfg_equal(const i2c_master_bus_config_t *a, const i2c_master_bus_config_t *b)
{
    return a->scl_io_num == b->scl_io_num
        && a->sda_io_num == b->sda_io_num
        && a->flags.enable_internal_pullup == b->flags.enable_internal_pullup;
}

static esp_err_t i2c_setup_device(i2c_dev_t *dev)
{
    return i2c_master_bus_add_device(dev->bus, &dev->device_config, &dev->device);
}

static esp_err_t i2c_setup_port(i2c_dev_t *dev)
{
    if (dev->port >= I2C_NUM_MAX) return ESP_ERR_INVALID_ARG;

    esp_err_t res;
    if (!cfg_equal(&dev->master_bus_config, &states[dev->port].config) || !states[dev->port].installed)
    {
        ESP_LOGD(TAG, "Reconfiguring I2C driver on port %d", dev->port);
        i2c_master_bus_config_t temp;
        memcpy(&temp, &dev->master_bus_config, sizeof(i2c_master_bus_config_t));

        // Driver reinstallation
        if (states[dev->port].installed)
        {
            if (!dev->bus)
            {
                ESP_LOGW(TAG, "The state of I2C port %d is `installed`, but bus handle is NULL)", dev->port);
            }
            else
            {
                i2c_del_master_bus(dev->bus);
            }
            states[dev->port].installed = false;
        }
        if ((res = i2c_new_master_bus(&dev->master_bus_config, &dev->bus)) != ESP_OK)
            return res;
        states[dev->port].bus = dev->bus;
        states[dev->port].installed = true;

        memcpy(&states[dev->port].config, &temp, sizeof(i2c_master_bus_config_t));
        ESP_LOGD(TAG, "I2C driver successfully reconfigured on port %d", dev->port);
    }
    return i2c_setup_device(dev);
}

esp_err_t _i2cdev_init()
{
    memset(states, 0, sizeof(states));

#if !CONFIG_I2CDEV_NOLOCK
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        states[i].lock = xSemaphoreCreateMutex();
        if (!states[i].lock)
        {
            ESP_LOGE(TAG, "Could not create port mutex %d", i);
            return ESP_FAIL;
        }
    }
#endif

    return ESP_OK;
}

esp_err_t _i2cdev_done()
{
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        if (!states[i].lock) continue;

        if (states[i].installed)
        {
            SEMAPHORE_TAKE(i);
            if (!states[i].bus)
            {
                ESP_LOGW(TAG, "The state of I2C port %d is `installed`, but bus handle is NULL)", i);
            }
            else
            {
                i2c_del_master_bus(states[i].bus);
            }
            states[i].installed = false;
            SEMAPHORE_GIVE(i);
        }
#if !CONFIG_I2CDEV_NOLOCK
        vSemaphoreDelete(states[i].lock);
#endif
        states[i].lock = NULL;
    }
    return ESP_OK;
}

esp_err_t _i2c_dev_probe(i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        res = i2c_master_probe(dev->bus, dev->device_config.device_address, CONFIG_I2CDEV_TIMEOUT);
    }

    SEMAPHORE_GIVE(dev->port);

    return res;
}

esp_err_t _i2c_dev_read(i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
    if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res == ESP_OK)
    {

        if (out_data && out_size)
        {
            res = i2c_master_transmit_receive(dev->device, (void *)out_data, out_size, in_data, in_size, CONFIG_I2CDEV_TIMEOUT);
        }
        else
        {
            res = i2c_master_receive(dev->device, (void *)out_data, out_size, CONFIG_I2CDEV_TIMEOUT);
        }
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
        }
    }

    SEMAPHORE_GIVE(dev->port);
    return res;
}

esp_err_t _i2c_dev_write(i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
    if (!dev || !out_data || !out_size) return ESP_ERR_INVALID_ARG;
    uint8_t *write_buffer = NULL;
    size_t write_buffer_size = 0;
    esp_err_t res = ESP_FAIL;
    bool with_register_address = out_reg && out_reg_size;

    SEMAPHORE_TAKE(dev->port);

    write_buffer_size = with_register_address ? out_reg_size + out_size : out_size;
    write_buffer = malloc(write_buffer_size);
    if (!write_buffer)
    {
        ESP_LOGE(TAG, "_i2c_dev_write: malloc() failed: size %d", write_buffer_size);
        res = ESP_ERR_NO_MEM;
        goto fail;
    }
    res = i2c_setup_port(dev);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_setup_port(): %s", esp_err_to_name(res));
        goto fail;
    }

    if (with_register_address)
    {
        memcpy(write_buffer, out_reg, out_reg_size);
        memcpy(write_buffer + out_reg_size, out_data, out_size);

    }
    else
    {
        memcpy(write_buffer, out_data, out_size);
    }
    res = i2c_master_transmit(dev->device, write_buffer, out_reg_size + out_size, CONFIG_I2CDEV_TIMEOUT);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
        goto fail;
    }
fail:
    free(write_buffer);
    SEMAPHORE_GIVE(dev->port);
    return res;
}

#include <string.h>
#include <esp_log.h>

#include "i2cdev_legacy.h"
#include "i2cdev_lock.h"

static const char *TAG = "i2cdev";

static i2c_port_state_t states[I2C_NUM_MAX];

inline static bool cfg_equal(const i2c_config_t *a, const i2c_config_t *b)
{
    return a->scl_io_num == b->scl_io_num
        && a->sda_io_num == b->sda_io_num
#if HELPER_TARGET_IS_ESP32
        && a->master.clk_speed == b->master.clk_speed
#elif HELPER_TARGET_IS_ESP8266
        && ((a->clk_stretch_tick && a->clk_stretch_tick == b->clk_stretch_tick) 
            || (!a->clk_stretch_tick && b->clk_stretch_tick == I2CDEV_MAX_STRETCH_TIME)
        ) // see line 232
#endif
        && a->scl_pullup_en == b->scl_pullup_en
        && a->sda_pullup_en == b->sda_pullup_en;
}

static esp_err_t i2c_setup_port(const i2c_dev_t *dev)
{
    if (dev->port >= I2C_NUM_MAX) return ESP_ERR_INVALID_ARG;

    esp_err_t res;
    if (!cfg_equal(&dev->cfg, &states[dev->port].config) || !states[dev->port].installed)
    {
        ESP_LOGD(TAG, "Reconfiguring I2C driver on port %d", dev->port);
        i2c_config_t temp;
        memcpy(&temp, &dev->cfg, sizeof(i2c_config_t));
        temp.mode = I2C_MODE_MASTER;

        // Driver reinstallation
        if (states[dev->port].installed)
        {
            i2c_driver_delete(dev->port);
            states[dev->port].installed = false;
        }
#if HELPER_TARGET_IS_ESP32
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
        // See https://github.com/espressif/esp-idf/issues/10163
        if ((res = i2c_driver_install(dev->port, temp.mode, 0, 0, 0)) != ESP_OK)
            return res;
        if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
            return res;
#else
        if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
            return res;
        if ((res = i2c_driver_install(dev->port, temp.mode, 0, 0, 0)) != ESP_OK)
            return res;
#endif
#endif
#if HELPER_TARGET_IS_ESP8266
        // Clock Stretch time, depending on CPU frequency
        temp.clk_stretch_tick = dev->timeout_ticks ? dev->timeout_ticks : I2CDEV_MAX_STRETCH_TIME;
        if ((res = i2c_driver_install(dev->port, temp.mode)) != ESP_OK)
            return res;
        if ((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
            return res;
#endif
        states[dev->port].installed = true;

        memcpy(&states[dev->port].config, &temp, sizeof(i2c_config_t));
        ESP_LOGD(TAG, "I2C driver successfully reconfigured on port %d", dev->port);
    }
#if HELPER_TARGET_IS_ESP32
    int t;
    if ((res = i2c_get_timeout(dev->port, &t)) != ESP_OK)
        return res;
    // Timeout cannot be 0
    uint32_t ticks = dev->timeout_ticks ? dev->timeout_ticks : I2CDEV_MAX_STRETCH_TIME;
    if ((ticks != t) && (res = i2c_set_timeout(dev->port, ticks)) != ESP_OK)
        return res;
    ESP_LOGD(TAG, "Timeout: ticks = %" PRIu32 " (%" PRIu32 " usec) on port %d", dev->timeout_ticks, dev->timeout_ticks / 80, dev->port);
#endif

    return ESP_OK;
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
            i2c_driver_delete(i);
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

esp_err_t _i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev->addr << 1 | (operation_type == I2C_DEV_READ ? 1 : 0), true);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));

        i2c_cmd_link_delete(cmd);
    }

    SEMAPHORE_GIVE(dev->port);

    return res;
}

esp_err_t _i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
    if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (out_data && out_size)
        {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, dev->addr << 1, true);
            i2c_master_write(cmd, (void *)out_data, out_size, true);
        }
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true);
        i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
        if (res != ESP_OK)
            ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));

        i2c_cmd_link_delete(cmd);
    }

    SEMAPHORE_GIVE(dev->port);
    return res;
}

esp_err_t _i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
    if (!dev || !out_data || !out_size) return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev->addr << 1, true);
        if (out_reg && out_reg_size)
            i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
        i2c_master_write(cmd, (void *)out_data, out_size, true);
        i2c_master_stop(cmd);
        res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
        if (res != ESP_OK)
            ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
        i2c_cmd_link_delete(cmd);
    }

    SEMAPHORE_GIVE(dev->port);
    return res;
}

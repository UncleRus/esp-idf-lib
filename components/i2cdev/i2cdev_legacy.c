/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file i2cdev.c
 *
 * ESP-IDF I2C master thread-safe functions for communication with I2C slave
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Updated 2025 by quinkq to use newer ESP-IDF I2C master driver API
 * MIT Licensed as described in the file LICENSE
 */
#include "esp_idf_lib_helpers.h" // For HELPER_TARGET_IS_ESP32 etc.
#include "i2cdev.h"              // Common header
#include <driver/i2c.h>          // Legacy I2C driver
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <sdkconfig.h>
#if !HELPER_TARGET_IS_ESP8266
#include <soc/clk_tree_defs.h> // For APB_CLK_FREQ
#endif
#include <string.h>

static const char *TAG = "i2cdev_legacy";

typedef struct
{
    SemaphoreHandle_t lock;
    i2c_config_t config; // Use legacy config struct
    bool installed;
    uint32_t ref_count;
    i2c_dev_t *devices[CONFIG_I2CDEV_MAX_DEVICES_PER_PORT]; // Track devices registered on this port
} i2c_port_state_t;

static i2c_port_state_t states[I2C_NUM_MAX] = {0};

#if CONFIG_I2CDEV_NOLOCK
#define SEMAPHORE_TAKE(port)
#else
#define SEMAPHORE_TAKE(port)                                                          \
    do                                                                                \
    {                                                                                 \
        if (!xSemaphoreTake(states[port].lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT))) \
        {                                                                             \
            ESP_LOGE(TAG, "Could not take port mutex %d", port);                      \
            return ESP_ERR_TIMEOUT;                                                   \
        }                                                                             \
    } while (0)
#endif

#if CONFIG_I2CDEV_NOLOCK
#define SEMAPHORE_GIVE(port)
#else
#define SEMAPHORE_GIVE(port)                                     \
    do                                                           \
    {                                                            \
        if (!xSemaphoreGive(states[port].lock))                  \
        {                                                        \
            ESP_LOGE(TAG, "Could not give port mutex %d", port); \
            return ESP_FAIL;                                     \
        }                                                        \
    } while (0)
#endif

/**
 * @brief Register an I2C device for tracking and resource management
 *
 * This function adds a device to the port's tracking array, which helps with:
 * - Monitoring which devices are active on each port
 * - Proper cleanup when the system shuts down
 * - Diagnostics and debugging
 *
 * Each port can track up to CONFIG_I2CDEV_MAX_DEVICES_PER_PORT devices.
 *
 * @param dev Device descriptor to register
 * @return ESP_OK if registration succeeded, or an error code
 */
static esp_err_t register_device(i2c_dev_t *dev)
{
    if (!dev || dev->port >= I2C_NUM_MAX)
        return ESP_ERR_INVALID_ARG;
    if (!states[dev->port].lock)
        return ESP_ERR_INVALID_STATE;

    esp_err_t ret = ESP_ERR_NO_MEM;

    // Take the mutex directly instead of using the macro
    if (xSemaphoreTake(states[dev->port].lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)) != pdTRUE)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not take port mutex for registration", dev->addr, dev->port);
        return ESP_ERR_TIMEOUT;
    }

    // Search for an empty slot in the device tracking array
    for (int i = 0; i < CONFIG_I2CDEV_MAX_DEVICES_PER_PORT; i++)
    {
        if (states[dev->port].devices[i] == NULL)
        {
            // Found empty slot - register the device here
            states[dev->port].devices[i] = dev;
            ESP_LOGV(TAG, "[0x%02x at %d] Registered device in slot %d", dev->addr, dev->port, i);
            ret = ESP_OK;
            break;
        }
    }

    // All slots full - this will still allow communication but prevents automatic cleanup
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "[0x%02x at %d] No free slots to register device", dev->addr, dev->port);
    }

    // Release the mutex
    if (!xSemaphoreGive(states[dev->port].lock))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not give port mutex after registration", dev->addr, dev->port);
        // If can't give the mutex, that's a serious error that overrides the registration result
        return ESP_FAIL;
    }

    return ret;
}

/**
 * @brief Deregister a device and update reference counting
 *
 * This function:
 * 1. Removes the device from the port's tracking array
 * 2. Decrements the port's reference count
 * 3. Cleans up the I2C driver if this was the last device on the port
 *
 * This is called during device cleanup to ensure proper resource management.
 *
 * @param dev Device descriptor to deregister
 */
static void deregister_device(i2c_dev_t *dev)
{
    if (!dev || dev->port >= I2C_NUM_MAX)
        return;

    // Don't use macros that return values since this is a void function
    if (states[dev->port].lock)
    {
        if (xSemaphoreTake(states[dev->port].lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)) != pdTRUE)
        {
            ESP_LOGE(TAG, "[0x%02x at %d] Could not take port mutex for deregistration", dev->addr, dev->port);
            return; // Cannot proceed without lock
        }

        // Find the device in the tracking array
        for (int i = 0; i < CONFIG_I2CDEV_MAX_DEVICES_PER_PORT; i++)
        {
            if (states[dev->port].devices[i] == dev)
            {
                // Clear this slot
                states[dev->port].devices[i] = NULL;
                ESP_LOGV(TAG, "[0x%02x at %d] Deregistered device from slot %d", dev->addr, dev->port, i);
                break;
            }
        }

        // Manage reference counting for this port
        if (states[dev->port].ref_count > 0)
        {
            states[dev->port].ref_count--;
            ESP_LOGD(TAG, "[Port %d] Decremented ref_count to %" PRIu32, dev->port, states[dev->port].ref_count);

            // If this was the last device using this port, clean up the driver
            if (states[dev->port].ref_count == 0 && states[dev->port].installed)
            {
                ESP_LOGI(TAG, "[Port %d] Last device removed, uninstalling driver", dev->port);
                i2c_driver_delete(dev->port);
                states[dev->port].installed = false;
            }
        }

        // Release the mutex
        if (!xSemaphoreGive(states[dev->port].lock))
        {
            ESP_LOGE(TAG, "[Port %d] Could not give port mutex after deregistration", dev->port);
            // Can't do much about this error except log it
        }
    }
}

esp_err_t i2cdev_init()
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

esp_err_t i2cdev_done()
{
    ESP_LOGV(TAG, "Cleaning up I2C subsystem (legacy)...");
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        if (!states[i].lock)
            continue;

        if (states[i].installed)
        {
            SEMAPHORE_TAKE(i);

            // First, clean up any devices still registered on this port
            for (int j = 0; j < CONFIG_I2CDEV_MAX_DEVICES_PER_PORT; j++)
            {
                if (states[i].devices[j] != NULL)
                {
                    i2c_dev_t *dev = states[i].devices[j];
                    ESP_LOGW(TAG, "[Port %d] Device 0x%02x still registered during cleanup", i, dev->addr);
                    states[i].devices[j] = NULL;
                }
            }

            i2c_driver_delete(i);
            states[i].installed = false;
            states[i].ref_count = 0;

            SEMAPHORE_GIVE(i);
        }
#if !CONFIG_I2CDEV_NOLOCK
        vSemaphoreDelete(states[i].lock);
#endif
        states[i].lock = NULL;
    }
    ESP_LOGV(TAG, "I2C subsystem cleanup finished (legacy).");
    return ESP_OK;
}

esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Creating device mutex", dev->addr, dev->port);

    // Initialize device pins to -1 to ensure consistent pattern with new driver
    if (dev->sda_pin == 0 && dev->scl_pin == 0)
    {
        dev->sda_pin = -1;
        dev->scl_pin = -1;
        ESP_LOGD(TAG, "[0x%02x at %d] Initialized pins to -1", dev->addr, dev->port);
    }

    if (dev->mutex)
    {
        ESP_LOGW(TAG, "[0x%02x at %d] Device mutex already exists", dev->addr, dev->port);
        return ESP_OK; // Already created
    }

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not create device mutex", dev->addr, dev->port);
        return ESP_FAIL;
    }

    // Register device for tracking
    esp_err_t reg_res = register_device(dev);
    if (reg_res != ESP_OK)
    {
        ESP_LOGW(TAG, "[0x%02x at %d] Could not register device: %s", dev->addr, dev->port, esp_err_to_name(reg_res));
        // Continue anyway since this is not critical
    }
#endif

    return ESP_OK;
}

esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Deleting device mutex and cleaning up", dev->addr, dev->port);

    // Deregister and update ref counts
    deregister_device(dev);

    // Delete mutex if exists
    if (dev->mutex)
    {
        vSemaphoreDelete(dev->mutex);
        dev->mutex = NULL;
    }
    else
    {
        ESP_LOGV(TAG, "[0x%02x at %d] Device mutex was NULL", dev->addr, dev->port);
    }
#endif
    return ESP_OK;
}

esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Taking mutex", dev->addr, dev->port);

    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Attempt to take NULL mutex!", dev->addr, dev->port);
        return ESP_ERR_INVALID_STATE;
    }

    if (!xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT)))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not take device mutex (timeout %d ms)", dev->addr, dev->port,
                 CONFIG_I2CDEV_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }
#endif
    return ESP_OK;
}

esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev)
{
#if !CONFIG_I2CDEV_NOLOCK
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Giving mutex", dev->addr, dev->port);

    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Attempt to give NULL mutex!", dev->addr, dev->port);
        return ESP_ERR_INVALID_STATE;
    }

    if (!xSemaphoreGive(dev->mutex))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Could not give device mutex", dev->addr, dev->port);
        return ESP_FAIL;
    }
#endif
    return ESP_OK;
}

inline static bool cfg_equal(const i2c_config_t *a, const i2c_config_t *b)
{
    bool clock_equal;
#ifdef CONFIG_IDF_TARGET_ESP8266
    clock_equal = (a->clk_stretch_tick == b->clk_stretch_tick);
#else
    clock_equal = (a->master.clk_speed == b->master.clk_speed);
#endif

    return a->mode == b->mode && a->scl_io_num == b->scl_io_num && a->sda_io_num == b->sda_io_num &&
           a->scl_pullup_en == b->scl_pullup_en && a->sda_pullup_en == b->sda_pullup_en && clock_equal;
    // Note: Ignoring clk_flags for comparison as it might not be consistently set by users
}

/**
 * @brief Configure and initialize the I2C port for a device
 *
 * This function is responsible for:
 * 1. Determining which pins to use (from device or config)
 * 2. Validating pin configuration
 * 3. Installing/configuring the I2C driver if not already done
 * 4. Managing reference counting for the port
 * 5. Setting up clock stretching timeout
 *
 * This is a critical function that must succeed before any I2C operations
 * can be performed with a device.
 *
 * @param dev Device descriptor with configuration info
 * @return ESP_OK on success, or an error code on failure
 */
static esp_err_t i2c_setup_port(i2c_dev_t *dev)
{
    if (!dev)
    {
        ESP_LOGE(TAG, "Device is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->port >= I2C_NUM_MAX)
    {
        ESP_LOGE(TAG, "Invalid I2C port number: %d", dev->port);
        return ESP_ERR_INVALID_ARG;
    }

    // Pin Selection Logic:
    // Pins are taken from dev->cfg.xyz_io_num.
    // If -1, Kconfig defaults are used.
    gpio_num_t sda_pin; // Effective SDA pin to be used
    gpio_num_t scl_pin; // Effective SCL pin to be used

    if (dev->cfg.sda_io_num == (gpio_num_t)-1)
    {
        sda_pin = (gpio_num_t)CONFIG_I2CDEV_DEFAULT_SDA_PIN;
    }
    else
    {
        sda_pin = dev->cfg.sda_io_num;
    }

    if (dev->cfg.scl_io_num == (gpio_num_t)-1)
    {
        scl_pin = (gpio_num_t)CONFIG_I2CDEV_DEFAULT_SCL_PIN;
    }
    else
    {
        scl_pin = dev->cfg.scl_io_num;
    }

    ESP_LOGD(TAG, "[0x%02x at %d] Based on cfg: sda_cfg=%d, scl_cfg=%d. Effective pins for setup: SDA=%d, SCL=%d", dev->addr,
             dev->port, dev->cfg.sda_io_num, dev->cfg.scl_io_num, sda_pin, scl_pin);

    // Perform basic validation of effective pins
    if (sda_pin < 0 || scl_pin < 0)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Invalid effective SDA/SCL pins (%d, %d). Check Kconfig defaults if cfg pins were -1.",
                 dev->addr, dev->port, sda_pin, scl_pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (sda_pin == scl_pin)
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Effective SDA and SCL pins cannot be the same (%d).", dev->addr, dev->port, sda_pin);
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize common fields
    i2c_config_t legacy_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin, // Use locally determined pins
        .scl_io_num = scl_pin, // Use locally determined pins
        .sda_pullup_en = dev->cfg.sda_pullup_en ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .scl_pullup_en = dev->cfg.scl_pullup_en ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE};

#ifdef CONFIG_IDF_TARGET_ESP8266
    // ESP8266 uses clk_stretch_tick instead of master.clk_speed
    // Clock speed will be handled during driver installation
    uint32_t desired_speed = dev->cfg.master.clk_speed > 0 ? dev->cfg.master.clk_speed : 400000;
    ESP_LOGD(TAG, "Final I2C config for port %d: SDA=%d, SCL=%d, speed=%lu (ESP8266)", dev->port, legacy_cfg.sda_io_num,
             legacy_cfg.scl_io_num, (unsigned long)desired_speed);
#else
    // ESP32 family uses master.clk_speed
    legacy_cfg.master.clk_speed = dev->cfg.master.clk_speed > 0 ? dev->cfg.master.clk_speed : 400000;
    ESP_LOGD(TAG, "Final I2C config for port %d: SDA=%d, SCL=%d, speed=%lu", dev->port, legacy_cfg.sda_io_num,
             legacy_cfg.scl_io_num, (unsigned long)legacy_cfg.master.clk_speed);
#endif

#ifdef CONFIG_IDF_TARGET_ESP32
    legacy_cfg.clk_flags = 0;
#endif

    esp_err_t err = ESP_OK;

    // Part 1: Driver Installation / Reconfiguration
    if (!cfg_equal(&legacy_cfg, &states[dev->port].config) || !states[dev->port].installed)
    {
        ESP_LOGD(TAG, "[0x%02x at %d] Reconfiguring I2C driver", dev->addr, dev->port);

        if (states[dev->port].installed)
        {
            ESP_LOGD(TAG, "Uninstalling previous I2C driver configuration for port %d", dev->port);
            i2c_driver_delete(dev->port);
            states[dev->port].installed = false;
            states[dev->port].ref_count = 0;
        }

        vTaskDelay(1);

// Target-specific driver installation/configuration sequence
#if HELPER_TARGET_IS_ESP32 || HELPER_TARGET_IS_ESP32S2 || HELPER_TARGET_IS_ESP32S3 || HELPER_TARGET_IS_ESP32C3 || \
    HELPER_TARGET_IS_ESP32C6
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
        ESP_LOGD(TAG, "Using IDF >= 5.1.0 driver install order for ESP32 family");
        err = i2c_driver_install(dev->port, legacy_cfg.mode, 0, 0, 0);
        if (err == ESP_OK)
        {
            err = i2c_param_config(dev->port, &legacy_cfg);
        }
#else
        ESP_LOGD(TAG, "Using IDF < 5.1.0 driver install order for ESP32 family");
        err = i2c_param_config(dev->port, &legacy_cfg);
        if (err == ESP_OK)
        {
            err = i2c_driver_install(dev->port, legacy_cfg.mode, 0, 0, 0);
        }
#endif
#elif HELPER_TARGET_IS_ESP8266
        ESP_LOGD(TAG, "Using ESP8266 specific driver installation");
        legacy_cfg.clk_stretch_tick = dev->timeout_ticks ? dev->timeout_ticks : I2CDEV_MAX_STRETCH_TIME;
        err = i2c_driver_install(dev->port, legacy_cfg.mode);
        if (err == ESP_OK)
        {
            err = i2c_param_config(dev->port, &legacy_cfg);
        }
        // ESP8266 note: Clock speed is not directly configurable through i2c_config_t
        // The desired speed was: %lu Hz", desired_speed
#else
        // If legacy mode is off, and target detection fails, this avoids a compile error.
        // The legacy driver just won't support any target in this case.
        ESP_LOGW(TAG, "i2cdev_legacy.c: No specific target (ESP32/ESP32-S2/ESP32-S3/ESP32-C3/ESP32-C6/ESP8266) detected "
                      "for driver installation. Legacy driver might be inactive or misconfigured.");
        err = ESP_ERR_NOT_SUPPORTED; // Indicate that setup can't proceed.
#endif

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to install/configure I2C driver for port %d: %d (%s)", dev->port, err,
                     esp_err_to_name(err));
            states[dev->port].installed = false; // Ensure state reflects failure
            return err;
        }

        memcpy(&states[dev->port].config, &legacy_cfg, sizeof(i2c_config_t));
        states[dev->port].installed = true;
        states[dev->port].ref_count++;

        dev->sda_pin = legacy_cfg.sda_io_num;
        dev->scl_pin = legacy_cfg.scl_io_num;

        ESP_LOGD(TAG, "I2C driver successfully installed/reconfigured on port %d, ref_count=%" PRIu32, dev->port,
                 states[dev->port].ref_count);
    }
    else
    {
        states[dev->port].ref_count++;
        ESP_LOGV(TAG, "I2C driver already installed on port %d with matching config, ref_count=%" PRIu32, dev->port,
                 states[dev->port].ref_count);

        dev->sda_pin = states[dev->port].config.sda_io_num;
        dev->scl_pin = states[dev->port].config.scl_io_num;
    }

// Part 2: Timeout Configuration (ESP32 family specific hardware timeout)
#if HELPER_TARGET_IS_ESP32 || HELPER_TARGET_IS_ESP32S2 || HELPER_TARGET_IS_ESP32S3 || HELPER_TARGET_IS_ESP32C3 || \
    HELPER_TARGET_IS_ESP32C6
    int current_timeout_hw;
    err = i2c_get_timeout(dev->port, &current_timeout_hw);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get HW timeout for port %d: %d (%s)", dev->port, err, esp_err_to_name(err));
        return err;
    }
    uint32_t timeout_ticks_val = dev->timeout_ticks ? dev->timeout_ticks : I2CDEV_MAX_STRETCH_TIME;
    if (timeout_ticks_val != (uint32_t)current_timeout_hw)
    {
        ESP_LOGV(TAG, "Port %d: Updating HW timeout from %d to %" PRIu32 " ticks", dev->port, current_timeout_hw,
                 timeout_ticks_val);
        err = i2c_set_timeout(dev->port, timeout_ticks_val);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set HW timeout for port %d: %d (%s)", dev->port, err, esp_err_to_name(err));
            return err;
        }
        ESP_LOGD(TAG, "HW Timeout: ticks = %" PRIu32 " (%" PRIu32 " usec) on port %d", timeout_ticks_val,
                 timeout_ticks_val / 80, dev->port);
    }
#endif

    return ESP_OK;
}

esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    esp_err_t res = i2c_setup_port((i2c_dev_t *)dev);
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

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
    if (!dev || !in_data || !in_size)
        return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    // Use a local status variable to track errors
    esp_err_t err = i2c_setup_port((i2c_dev_t *)dev);
    if (err == ESP_OK)
    {
        // Only create a command handle if setup was successful
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        if (out_data && out_size)
        {
            // Write phase - typically used to specify a register address
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, dev->addr << 1, true); // Addr + Write bit (0)
            i2c_master_write(cmd, (void *)out_data, out_size, true);
        }

        // Read phase - get data from the device
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true); // Addr + Read bit (1)
        i2c_master_read(cmd, in_data, in_size,
                        I2C_MASTER_LAST_NACK); // NACK the last byte to signal end
        i2c_master_stop(cmd);

        // Execute the command
        err = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "i2c_master_cmd_begin failed for read: %d (%s)", err, esp_err_to_name(err));
        }

        // Always delete the command handle
        i2c_cmd_link_delete(cmd);
    }

    // Always release the semaphore before returning
    SEMAPHORE_GIVE(dev->port);
    return err;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data,
                        size_t out_size)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    if ((!out_reg || !out_reg_size) && (!out_data || !out_size))
        return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    // Use a local status variable to track errors
    esp_err_t err = i2c_setup_port((i2c_dev_t *)dev);
    if (err == ESP_OK)
    {
        // Only create a command handle if setup was successful
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev->addr << 1, true);

        // Write register address/command if provided
        if (out_reg && out_reg_size)
        {
            i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
        }

        // Write data if provided
        if (out_data && out_size)
        {
            i2c_master_write(cmd, (void *)out_data, out_size, true);
        }

        i2c_master_stop(cmd);

        // Execute the command
        err = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "i2c_master_cmd_begin failed for write: %d (%s)", err, esp_err_to_name(err));
        }

        // Always delete the command handle
        i2c_cmd_link_delete(cmd);
    }

    // Always release the semaphore before returning
    SEMAPHORE_GIVE(dev->port);
    return err;
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size)
{
    if (!dev || !out_data || !out_size)
        return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    // Use a local status variable to track errors
    esp_err_t err = i2c_setup_port((i2c_dev_t *)dev);
    if (err == ESP_OK)
    {
        // Only create a command handle if setup was successful
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true); // Addr + Write bit
        i2c_master_write_byte(cmd, reg, true);                                 // Register address
        if (out_data && out_size)
        {
            i2c_master_write(cmd, (void *)out_data, out_size, true); // Data to write
        }
        i2c_master_stop(cmd);

        // Execute the command
        err = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "i2c_master_cmd_begin failed for write_reg: %d (%s)", err, esp_err_to_name(err));
        }

        // Always delete the command handle
        i2c_cmd_link_delete(cmd);
    }

    // Always release the semaphore before returning
    SEMAPHORE_GIVE(dev->port);
    return err;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
    if (!dev || !in_data || !in_size)
        return ESP_ERR_INVALID_ARG;

    SEMAPHORE_TAKE(dev->port);

    // Use a local status variable to track errors
    esp_err_t err = i2c_setup_port((i2c_dev_t *)dev);
    if (err == ESP_OK)
    {
        // Only create a command handle if setup was successful
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        // Execute the command
        err = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "i2c_master_cmd_begin failed for read_reg: %d (%s)", err, esp_err_to_name(err));
        }

        // Always delete the command handle
        i2c_cmd_link_delete(cmd);
    }

    // Always release the semaphore before returning
    SEMAPHORE_GIVE(dev->port);
    return err;
}

// Implementation of i2c_dev_check_present using legacy I2C driver
esp_err_t i2c_dev_check_present(const i2c_dev_t *dev)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[0x%02x at %d] Checking device presence (legacy driver)...", dev->addr, dev->port);

    // Use the existing probe function with write operation
    return i2c_dev_probe(dev, I2C_DEV_WRITE);
}

#if !defined(__I2CDEV_LOCK_H__)
#define __I2CDEV_LOCK_H__

#if CONFIG_I2CDEV_NOLOCK
#define SEMAPHORE_GIVE(port)
#else
#define SEMAPHORE_GIVE(port) do { \
        if (!xSemaphoreGive(states[port].lock)) \
        { \
            ESP_LOGE(TAG, "Could not give port mutex %d", port); \
            return ESP_FAIL; \
        } \
        } while (0)
#endif

#if CONFIG_I2CDEV_NOLOCK
#define SEMAPHORE_TAKE(port)
#else
#define SEMAPHORE_TAKE(port) do { \
        if (!xSemaphoreTake(states[port].lock, pdMS_TO_TICKS(CONFIG_I2CDEV_TIMEOUT))) \
        { \
            ESP_LOGE(TAG, "Could not take port mutex %d", port); \
            return ESP_ERR_TIMEOUT; \
        } \
        } while (0)
#endif

typedef struct {
    SemaphoreHandle_t lock;
#if defined(CONFIG_I2CDEV_USING_LEGACY_I2C)
    i2c_config_t config;
#else
    i2c_master_bus_config_t config;
    i2c_master_bus_handle_t bus;
#endif
    bool installed;
} i2c_port_state_t;

#endif

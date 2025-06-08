/**
 * @file i2cdev.h
 * @defgroup i2cdev i2cdev
 * @{
 *
 * ESP-IDF I2C master thread-safe functions for communication with I2C slave
 *
 * This implementation uses the newer ESP-IDF I2C master driver (v5.0+).
 * For ESP-IDF versions using the legacy I2C driver, use i2cdev_legacy.c instead.
 *
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Updated 2025 by quinkq to use newer ESP-IDF I2C master driver API
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Define missing types for older ESP-IDF versions
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
typedef enum
{
    I2C_ADDR_BIT_LEN_7 = 0, /*!< I2C 7bit address for slave mode */
    I2C_ADDR_BIT_LEN_10,    /*!< I2C 10bit address for slave mode */
} i2c_addr_bit_len_t;
#endif

// Definition for I2CDEV_MAX_STRETCH_TIME
#if HELPER_TARGET_IS_ESP8266
#define I2CDEV_MAX_STRETCH_TIME 0xffffffff
#else
#include <soc/i2c_reg.h> // For I2C_TIME_OUT_VALUE_V, etc.
#if defined(I2C_TIME_OUT_VALUE_V)
#define I2CDEV_MAX_STRETCH_TIME I2C_TIME_OUT_VALUE_V
#elif defined(I2C_TIME_OUT_REG_V)
#define I2CDEV_MAX_STRETCH_TIME I2C_TIME_OUT_REG_V
#else
#define I2CDEV_MAX_STRETCH_TIME 0x00ffffff
#endif
#endif /* HELPER_TARGET_IS_ESP8266 */

#ifndef CONFIG_I2CDEV_TIMEOUT
#define CONFIG_I2CDEV_TIMEOUT 1000 // Default 1 second timeout
#endif

#ifndef CONFIG_I2CDEV_NOLOCK
#define CONFIG_I2CDEV_NOLOCK 0 // Enable locking by default
#endif

#ifndef CONFIG_I2CDEV_MAX_DEVICES_PER_PORT
#define CONFIG_I2CDEV_MAX_DEVICES_PER_PORT 8 // Maximum devices per I2C port
#endif

#ifndef CONFIG_I2CDEV_DEFAULT_SDA_PIN
#define CONFIG_I2CDEV_DEFAULT_SDA_PIN 21 // Default SDA pin
#endif

#ifndef CONFIG_I2CDEV_DEFAULT_SCL_PIN
#define CONFIG_I2CDEV_DEFAULT_SCL_PIN 22 // Default SCL pin
#endif

#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100 // Default value in most ESP-IDF configs
#endif

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 3 // INFO level as default
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief I2C transaction type for legacy probe
     */
    typedef enum
    {
        I2C_DEV_WRITE = 0, /**< Write operation for probe */
        I2C_DEV_READ       /**< Read operation for probe */
    } i2c_dev_type_t;

    /**
     * I2C device descriptor
     *
     * @note Before calling i2c_dev_create_mutex(), initialize this structure with:
     *       - Port number in `dev->port`.
     *       - Device I2C address in `dev->addr`.
     *       - Address bit length (e.g., `I2C_ADDR_BIT_LEN_7`) in `dev->addr_bit_len`.
     *         If not set, defaults to 7-bit.
     *       - Desired SDA and SCL pins: Set these in `dev->cfg.sda_io_num` and `dev->cfg.scl_io_num`.
     *         Using -1 for these will make the library attempt to use Kconfig default pins
     *         (if this is the first device on the port) or the port\'s already configured pins.
     *         The actual pins used by the bus for this port will be populated by the library into
     *         `dev->sda_pin` and `dev->scl_pin` for informational purposes after port initialization.
     *       - Clock speed in `dev->cfg.master.clk_speed`.
     *       - Pullup settings in `dev->cfg.sda_pullup_en` and `dev->cfg.scl_pullup_en`.
     *       - Optionally, `dev->timeout_ticks` for custom clock stretching timeout (primarily for legacy driver).
     */
    typedef struct
    {
        i2c_port_t port;                 //!< I2C port number
        i2c_addr_bit_len_t addr_bit_len; //!< I2C address format (7-bit or 10-bit), defaults to I2C_ADDR_BIT_LEN_7 if not set
        uint16_t addr;                   //!< I2C address (7-bit address for I2C_ADDR_BIT_LEN_7)
        SemaphoreHandle_t mutex;         //!< Device mutex

        // The sda_pin and scl_pin are primarily for informational purposes after bus init or if
        // i2c_setup_port needs to check consistency. Drivers should set pins in cfg.sda_io_num and
        // cfg.scl_io_num.
        int sda_pin; //!< Actual SDA pin used by the bus for this device's port, populated after port init.
        int scl_pin; //!< Actual SCL pin used by the bus for this device's port, populated after port init.

        void *dev_handle;       //!< Device handle (implementation specific)
        uint32_t timeout_ticks; //!< Clock stretching timeout in ticks (for legacy driver)

        // Backward compatibility with original UncleRus implementation structure
        // Sensor drivers should populate these fields as they did previously.
        struct
        {
            gpio_num_t sda_io_num; //!< Desired GPIO number for SDA by driver (-1 if to use default)
            gpio_num_t scl_io_num; //!< Desired GPIO number for SCL by driver (-1 if to use default)
            uint8_t sda_pullup_en; //!< SDA line pullup enable
            uint8_t scl_pullup_en; //!< SCL line pullup enable
            struct
            {
                uint32_t clk_speed; //!< Desired clock speed in Hz for this device
            } master;               //!< I2C master configuration for this device
        } cfg;                      //!< I2C bus configuration parameters provided by the driver
    } i2c_dev_t;

    /**
     * @brief Initialize I2C subsystem (port mutexes and internal states)
     *
     * @note This should be called once at the beginning of your application
     * before any I2C devices are initialized.
     *
     * @return ESP_OK on success
     */
    esp_err_t i2cdev_init(void);

    /**
     * @brief Release I2C subsystem (deletes all devices, buses, and mutexes)
     *
     * @note Call this when no more I2C operations will be performed
     * to clean up resources.
     *
     * @return ESP_OK on success
     */
    esp_err_t i2cdev_done(void);

    /**
     * @brief Create mutex for device descriptor and register device
     *
     * @note IMPORTANT: Before calling this function, you must properly initialize the i2c_dev_t
     *       structure with device address, port, and pin settings. For ESP-IDF legacy driver,
     *       set pins in dev->cfg.sda_io_num and dev->cfg.scl_io_num. For newer ESP-IDF version,
     *       either method is compatible. See the structure documentation for details.
     *
     * @param dev Pointer to device descriptor
     * @return `ESP_OK` on success
     */
    esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev);

    /**
     * @brief Delete mutex for device descriptor and perform device cleanup
     *
     * @note This function performs cleanup tasks including removing the device from the
     *       I2C bus, deregistering it, and deleting its mutex.
     *
     * @param dev Pointer to device descriptor
     * @return `ESP_OK` on success
     */
    esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev);

    /**
     * @brief Take device mutex
     *
     * @param dev Pointer to device descriptor
     * @return `ESP_OK` on success
     */
    esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev);

    /**
     * @brief Give device mutex
     *
     * @param dev Pointer to device descriptor
     * @return `ESP_OK` on success
     */
    esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev);

    /**
     * @brief Check the availability of a device on the I2C bus (New Driver) - legacy's i2c_dev_probe function equivalent.
     *
     * This function attempts to communicate with the I2C device to see if it ACKs.
     * It is non-intrusive; if the device is found, any temporary setup for
     * the check is torn down. Uses the new I2C driver logic.
     *
     * @param dev Pointer to the device descriptor. Pins and address must be configured.
     * @return `ESP_OK` if the device ACKs (is present), an error code otherwise.
     */
    esp_err_t i2c_dev_check_present(const i2c_dev_t *dev);

    /**
     * @brief Check the availability of a device on the I2C bus (Legacy Driver).
     *
     * Issue an operation of `operation_type` to the I2C device then stops.
     * Primarily for use with the legacy i2cdev_legacy.c implementation.
     *
     * @param dev Device descriptor.
     * @param operation_type Operation type (I2C_DEV_WRITE or I2C_DEV_READ).
     * @return `ESP_OK` if device is available for the specified operation type.
     */
    esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type);

    /**
     * @brief Read from device
     *
     * @param dev Pointer to device descriptor
     * @param[in] out_data Data to write before reading (can be NULL if out_size is 0)
     * @param out_size Size of data to write
     * @param[out] in_data Buffer to store data read
     * @param in_size Number of bytes to read
     * @return `ESP_OK` on success
     */
    esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size);

    /**
     * @brief Write to device
     *
     * @param dev Pointer to device descriptor
     * @param[in] out_reg Register address to write to (can be NULL if out_reg_size is 0)
     * @param out_reg_size Size of register address
     * @param[in] out_data Data to write (can be NULL if out_size is 0)
     * @param out_size Size of data to write
     * @return `ESP_OK` on success
     */
    esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data,
                            size_t out_size);

    /**
     * @brief Read from device register (8-bit register address)
     *
     * @param dev Pointer to device descriptor
     * @param reg Command to write before reading
     * @param[out] data Buffer to store data
     * @param size Number of bytes to read
     * @return `ESP_OK` on success
     */
    esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *data, size_t size);

    /**
     * @brief Write to device register (8-bit register address)
     *
     * @param dev Pointer to device descriptor
     * @param reg Command to write before writing data
     * @param data Buffer with data to write
     * @param size Number of bytes to write
     * @return `ESP_OK` on success
     */
    esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *data, size_t size);

/**
 * @brief Take device mutex with error checking
 */
#define I2C_DEV_TAKE_MUTEX(dev)                 \
    do                                          \
    {                                           \
        esp_err_t __ = i2c_dev_take_mutex(dev); \
        if (__ != ESP_OK)                       \
            return __;                          \
    } while (0)

/**
 * @brief Give device mutex with error checking
 */
#define I2C_DEV_GIVE_MUTEX(dev)                 \
    do                                          \
    {                                           \
        esp_err_t __ = i2c_dev_give_mutex(dev); \
        if (__ != ESP_OK)                       \
            return __;                          \
    } while (0)

/**
 * @brief Execute operation, assuming mutex is held. Gives mutex ONLY on error.
 */
#define I2C_DEV_CHECK(dev, X)                       \
    do                                              \
    {                                               \
        esp_err_t ___ = X; /* Execute operation */  \
        if (___ != ESP_OK)                          \
        {                                           \
            /* Give mutex ONLY if error occurred */ \
            i2c_dev_give_mutex(dev);                \
            return ___;                             \
        }                                           \
    } while (0)

/**
 * @brief Execute operation, assuming mutex is held. Gives mutex ONLY on error, logs error.
 */
#define I2C_DEV_CHECK_LOGE(dev, X, msg, ...)        \
    do                                              \
    {                                               \
        esp_err_t ___ = X; /* Execute operation */  \
        if (___ != ESP_OK)                          \
        {                                           \
            /* Give mutex ONLY if error occurred */ \
            i2c_dev_give_mutex(dev);                \
            ESP_LOGE(TAG, msg, ##__VA_ARGS__);      \
            return ___;                             \
        }                                           \
    } while (0)

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __I2CDEV_H__ */

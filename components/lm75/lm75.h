#ifndef __LM75_H__
#define __LM75_H__

#include <stdint.h>
#include <stdbool.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LM75_I2C_ADDRESS_DEFAULT 0x48
#define LM75_I2C_ADDRESS_MAX 0x4f

#define LM75_OS_ACTIVE_LOW 0
#define LM75_OS_ACTIVE_HIGH 1
#define LM75_OS_COMPARATOR 0
#define LM75_OS_INTERRUPT 1

typedef enum {
    LM75_MODE_NORMAL = 0,
    LM75_MODE_SHUTDOWN = 1
} LM75_Mode;

typedef enum {
    LM75_OSP_LOW  = 0,
    LM75_OSP_HIGH = 1
} LM75_OS_Polarity;

typedef enum {
    LM75_OS_MODE_COMP = 0,
    LM75_OS_MODE_INT  = 1
} LM75_OS_Mode;

typedef struct {
    i2c_dev_t i2c_dev;
} lm75_t;

typedef enum {
    LM75_FAULT_QUEUE1 = 0b00,
    LM75_FAULT_QUEUE2 = 0b01,
    LM75_FAULT_QUEUE4 = 0b10,
    LM75_FAULT_QUEUE6 = 0b11
} LM75_Fault_Queue;

typedef struct {
    LM75_Mode mode;
    LM75_OS_Polarity os_pol;
    LM75_OS_Mode os_mode;
    LM75_Fault_Queue os_fault_queue;
} lm75_config_t;

esp_err_t lm75_read_temparature(lm75_t *dev, float *value);

esp_err_t lm75_init_desc(lm75_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

esp_err_t lm75_free_desc(lm75_t *dev);

esp_err_t lm75_init(lm75_t *dev, const lm75_config_t config);

esp_err_t lm75_shutdown(lm75_t *dev);

esp_err_t lm75_wakeup(lm75_t *dev);

esp_err_t lm75_set_os_threshold(lm75_t *dev, const float value);

esp_err_t lm75_get_os_threshold(lm75_t *dev, float *value);

esp_err_t lm75_set_os_polarity(lm75_t *dev, const uint8_t v);

esp_err_t lm75_get_os_polarity(lm75_t *dev, uint8_t *v);

esp_err_t lm75_set_os_mode(lm75_t *dev, const uint8_t v);

#ifdef __cplusplus
}
#endif

#endif

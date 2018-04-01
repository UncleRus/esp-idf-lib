/**
 * Driver for BH1750 light sensor
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2017 Andrej Krutak <dev@andree.sk>
 *               2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 *
 * ROHM Semiconductor bh1750fvi-e.pdf
 */
#ifndef EXTRAS_BH1750_H_
#define EXTRAS_BH1750_H_

#include <stdint.h>
#include <stdbool.h>
#include <i2c_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Possible chip addresses
 */
#define BH1750_ADDR_LO 0x23 //!< ADDR pin floating/low
#define BH1750_ADDR_HI 0x5c //!< ADDR pin high

/**
 * Measurement mode
 */
typedef enum
{
    BH1750_MODE_ONE_TIME = 0, //!< BH1750_MODE_ONE_TIME
    BH1750_MODE_CONTINIOUS    //!< BH1750_MODE_CONTINIOUS
} bh1750_mode_t;

/**
 * Measurement resolution
 */
typedef enum
{
    BH1750_RES_LOW = 0,  //!< 4 lx resolution, measurement time is typically 16 ms
    BH1750_RES_HIGH,     //!< 1 lx resolution, measurement time is typically 120 ms
    BH1750_RES_HIGH2     //!< 0.5 lx resolution, measurement time is typically 120 ms
} bh1750_resolution_t;

/**
 *
 * @param dev
 * @param mode
 * @param resolution
 * @return
 */
esp_err_t bh1750_setup(i2c_dev_t *dev, bh1750_mode_t mode, bh1750_resolution_t resolution);


/**
 * Read LUX value from the device.
 *
 * @param addr Device address
 * @returns read value in lux units
 */
esp_err_t bh1750_read(i2c_dev_t *dev, uint16_t *level);

#ifdef __cplusplus
}
#endif

#endif /* EXTRAS_BH1750_H_ */

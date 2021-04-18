#ifndef __NOISE_H__
#define __NOISE_H__

#include <lib8tion.h>

uint16_t inoise16_3d(uint32_t x, uint32_t y, uint32_t z);
uint16_t inoise16_2d(uint32_t x, uint32_t y);
uint16_t inoise16_1d(uint32_t x);

int16_t inoise16_3d_raw(uint32_t x, uint32_t y, uint32_t z);
int16_t inoise16_2d_raw(uint32_t x, uint32_t y);
int16_t inoise16_1d_raw(uint32_t x);

uint8_t inoise8_3d(uint16_t x, uint16_t y, uint16_t z);
uint8_t inoise8_2d(uint16_t x, uint16_t y);
uint8_t inoise8_1d(uint16_t x);

int8_t inoise8_3d_raw(uint16_t x, uint16_t y, uint16_t z);
int8_t inoise8_2d_raw(uint16_t x, uint16_t y);
int8_t inoise8_1d_raw(uint16_t x);

void fill_raw_noise8(uint8_t *pData, uint8_t num_points, uint8_t octaves, uint16_t x, int scale, uint16_t time);
void fill_raw_noise16into8(uint8_t *pData, uint8_t num_points, uint8_t octaves, uint32_t x, int scale, uint32_t time);

#endif /* __NOISE_H__ */

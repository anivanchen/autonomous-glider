#ifdef __cplusplus
extern "C" {
#endif

#ifndef __BMP280_LOW_LEVEL_H__
#define __BMP280_LOW_LEVEL_H__

#include "bmp280_defs.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdint.h>

void bmp280_write_array(uint8_t deviceAddress, uint8_t startRegisterAddress, uint8_t *data, uint8_t dataLength);
void bmp280_read_array(uint8_t deviceAddress, uint8_t startRegisterAddress, uint8_t *data, uint8_t dataLength);

void bmp280_i2c_init();
void delay_function(uint32_t delayMS);
float power_function(float x, float y);

#endif
#ifdef __cplusplus 
}
#endif